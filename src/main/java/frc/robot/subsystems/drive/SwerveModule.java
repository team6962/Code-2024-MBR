// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.Constants.Constants.SWERVE_DRIVE.DRIVE_MOTOR_PROFILE;
import frc.robot.Constants.Constants.SWERVE_DRIVE.MODULE_CONFIG;
import frc.robot.Constants.Constants.SWERVE_DRIVE.PHYSICS;
import frc.robot.Constants.Constants.SWERVE_DRIVE.STEER_MOTOR_PROFILE;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.MathUtils.SwerveMath;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor, steerMotor;                      // NEO motors controlled by CAN SparkMaxes
  private RelativeEncoder driveEncoder, steerEncoder;              // Built-in relative encoders from NEOs
  private CANcoder absoluteSteerEncoder;                           // Absolute encoder from CANCoder on steering shaft
  private SparkPIDController drivePID, steerPID;                   // Built-in PID controllers running on-board SparkMaxes at 1000 Hz
  private SwerveModuleState targetState = new SwerveModuleState(); // Target state for the module
  private int corner;                                              // Corner of the robot the module is mounted on (0: front-left, 1: front-right, 2: back-left, 3: back-right)
  private Rotation2d absoluteSteerDirection = new Rotation2d();    // Absolute direction of the steering motor, cached and taken from the CANCoder
  private double driveVelocity = 0.0;                              // Velocity of the wheel, cached and taken from the NEO's built-in encoder
  private double drivePosition = 0.0;                              // Meters the module has driven, cached and taken from the NEO's built-in encoder
  private Rotation2d relativeSteerDirection = new Rotation2d();    // Relative direction of the steering motor, cached and taken from the NEO's built-in encoder
  private boolean isCalibrating = false;                           // Whether the module is calibrating, used to prevent driving during calibration
  
  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward( // Feedforward Controller for the drive motor
    DRIVE_MOTOR_PROFILE.kS,
    DRIVE_MOTOR_PROFILE.kV,
    DRIVE_MOTOR_PROFILE.kA
  );

  public SwerveModule(MODULE_CONFIG config, int corner, String name) {
    this.corner = corner;

    driveMotor           = new CANSparkMax(config.CAN_DRIVE(), MotorType.kBrushless);
    steerMotor           = new CANSparkMax(config.CAN_STEER(), MotorType.kBrushless);
    absoluteSteerEncoder = new CANcoder(config.CAN_ENCODER());
    steerEncoder         = steerMotor.getEncoder();
    driveEncoder         = driveMotor.getEncoder();
    drivePID             = driveMotor.getPIDController();
    steerPID             = steerMotor.getPIDController();

    /*
     * Encoder offsets are determined by the physical mounting of the module on the robot.
     * This means that modules can be swapped between corners without needing to recalibrate the encoders.
     */
    double encoderOffset = config.ENCODER_OFFSET();
    switch (corner) {
      case 0:
        encoderOffset += 0.0;
        break;
      case 1:
        encoderOffset += 0.25;
        break;
      case 2:
        encoderOffset += -0.25;
        break;
      case 3:
        encoderOffset += 0.5;
        break;
      default:
    }
    encoderOffset %= 2; // Normalize the offset to be between 0 and 2
    encoderOffset = (encoderOffset > 1.0) ? encoderOffset - 2.0 : (encoderOffset < -1.0) ? encoderOffset + 2.0 : encoderOffset; // Normalize the offset to be between -1 and 1
    
    MagnetSensorConfigs magConfig = new MagnetSensorConfigs(); // CANCoder is configured with MagnetSensorConfigs object all at once
    magConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf); // Absolute sensor range is set to -180 to 180 degrees, instead of 0 to 360
    magConfig.withMagnetOffset(encoderOffset); // Magnet offset is set to the calculated encoder offset
    BaseStatusSignal.setUpdateFrequencyForAll(50, absoluteSteerEncoder.getAbsolutePosition(), absoluteSteerEncoder.getFaultField(), absoluteSteerEncoder.getVersion()); // CANCoder is set to update at 50 Hz, and only give data we care about
    absoluteSteerEncoder.optimizeBusUtilization(); // Apply bus frequency settings

    /*
     * Using custom logging tool, SparkMaxUtil, to configure and log the SparkMaxes and CANCoders.
     * The drive motor is set to have a calculated current limit that prevents the wheel from slipping by preventing the motor
     * from applying more torque than the friction of the wheel.
     * The steer motor is set to coast mode, as it doesn't need to hold position when disabled. This also prevents tearing up the carpet.
     */
    SparkMaxUtil.configureAndLog(this, driveMotor, false, CANSparkMax.IdleMode.kBrake, PHYSICS.SLIPLESS_CURRENT_LIMIT, PHYSICS.SLIPLESS_CURRENT_LIMIT);
    SparkMaxUtil.configureAndLog(this, steerMotor, true, CANSparkMax.IdleMode.kCoast);
    SparkMaxUtil.configureEncoder(driveMotor, SWERVE_DRIVE.DRIVE_ENCODER_CONVERSION_FACTOR); // Configure the drive encoder to be in meters rather than rotations
    SparkMaxUtil.configureEncoder(steerMotor, SWERVE_DRIVE.STEER_ENCODER_CONVERSION_FACTOR); // Configure the steer encoder to be in radians rather than rotations
    SparkMaxUtil.configurePID(this, driveMotor, DRIVE_MOTOR_PROFILE.kP, DRIVE_MOTOR_PROFILE.kI, DRIVE_MOTOR_PROFILE.kD, 0.0, false);
    SparkMaxUtil.configurePID(this, steerMotor, STEER_MOTOR_PROFILE.kP, STEER_MOTOR_PROFILE.kI, STEER_MOTOR_PROFILE.kD, 0.0, true); // Steer motor is continuous (-180 and 180 degrees wrap), so it needs to be configured as such
    
    // Save config and burn to flash
    SparkMaxUtil.save(driveMotor);
    SparkMaxUtil.save(steerMotor);
    
    // Reduce CAN bus utilization by only sending necessary data
    SparkMaxUtil.configureCANStatusFrames(driveMotor, true, true);
    SparkMaxUtil.configureCANStatusFrames(steerMotor, false, true);

    // Set the steer encoder in the motor to the absolute position of the CANCoder
    seedSteerEncoder();

    // Log the module's data
    String logPath = "module" + name + "/";
    Logger.autoLog(this, logPath + "relativeSteerDirection",           () -> relativeSteerDirection.getDegrees());
    Logger.autoLog(this, logPath + "absoluteSteerDirection",        () -> absoluteSteerDirection.getDegrees());

    // Add status checks for the module
    StatusChecks.addCheck(this, name + "canCoderHasFaults", () -> absoluteSteerEncoder.getFaultField().getValue() == 0);
    StatusChecks.addCheck(this, name + "canCoderIsConnected", () -> absoluteSteerEncoder.getVersion().getValue() != 0);
  }


  public void periodic() {
    /*
     * Update the module's state variables with the latest data from the SparkMaxes and CANCoders.
     * This is important because calling get() on the SparkMaxes and CANCoders is expensive, so we only want to do it once per loop.
     */
    relativeSteerDirection = Rotation2d.fromRadians(steerEncoder.getPosition());
    absoluteSteerDirection = Rotation2d.fromRotations(absoluteSteerEncoder.getAbsolutePosition().getValue());
    driveVelocity = driveEncoder.getVelocity();
    drivePosition = driveEncoder.getPosition();

    // Only drive the module if the system is enabled and the robot is not calibrating
    if (!ENABLED_SYSTEMS.ENABLE_DRIVE) return;
    if (isCalibrating) return;
    drive(targetState);

    // Stop motors if the battery voltage is too low
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SWERVE_DRIVE) stop();
  }
  
  public void drive(SwerveModuleState state) {
    double speedMetersPerSecond = state.speedMetersPerSecond;
    double radians = state.angle.getRadians();
    
    // Slow down the drive motor when the steering angle is far from the target angle.
    if (SWERVE_DRIVE.DO_ANGLE_ERROR_SPEED_REDUCTION) {
      speedMetersPerSecond *= Math.cos(SwerveMath.angleDistance(radians, getMeasuredState().angle.getRadians()));
    }
    
    // Using on-board SparkMax PID controllers to control the drive and steer motors. This allows for 1kHz closed loop control.
    drivePID.setReference(
      speedMetersPerSecond,
      CANSparkMax.ControlType.kVelocity,
      0,
      driveFF.calculate(speedMetersPerSecond) // Add our own feedforward instead of using the SparkMax's built-in feedforward. This is because the built-in feedforward does not support the kS term.
    );
    steerPID.setReference(
      radians,
      CANSparkMax.ControlType.kPosition
    );

    // If the module is not moving, seed the steer encoder with the absolute position of the steer CANCoder
    if (state.speedMetersPerSecond == 0 && Math.abs(getRelativeSteerDirection().minus(getAbsoluteSteerDirection()).getDegrees()) > 0.5) {
      seedSteerEncoder();
    }
  }
  
  /**
   * Sets the target state for the module to drive at.
   * @param state target state
   */
  public void setTargetState(SwerveModuleState state) {
    // Optimize the state to flip the steering angle if it is faster to go the other way. This ensures that the module always takes the shortest path to the target angle.
    targetState = SwerveModuleState.optimize(state, getMeasuredState().angle);
  }
  
  /**
   * Stops the module by setting the target state to 0 speed and the current angle.
   */
  public void stop() {
    targetState = new SwerveModuleState(0.0, getMeasuredState().angle);
  }
  
  /**
   * Seeds the position of the built-in relative encoder with the absolute position of the steer CANCoder.
   * This is because the CANCoder polls at a lower rate than we'd like, so we essentially turn the relative encoder into an fast-updating absolute encoder.
   * Also the built-in SparkMaxPIDControllers require a compatible encoder to run the faster 1kHz closed loop 
   */
  public void seedSteerEncoder() {
    steerEncoder.setPosition(getAbsoluteSteerDirection().getRadians());
  }

  /**
   * Gets the direction of the steering motor from the built-in relative encoder.
   * @return relative direction of the steering motor
   */
  public Rotation2d getRelativeSteerDirection() {
    return relativeSteerDirection;
  }
  
  /**
   * Gets the absolute direction of the steering motor from the CANCoder.
   * @return absolute direction of the steering motor
   */
  private Rotation2d getAbsoluteSteerDirection() {
    return absoluteSteerDirection;
  }

  /**
   * Gets the target state for the module to drive at.
   * @return target state
   */
  public SwerveModuleState getTargetState() {
    return targetState;
  }
  
  /**
   * Gets the measured state of the module, which is the current velocity and angle of the module.
   * @return measured state
   */
  public SwerveModuleState getMeasuredState() {
    return new SwerveModuleState(driveVelocity, getAbsoluteSteerDirection());
  }

  /**
   * Gets the position of the module, which is the distance the wheel has traveled and the angle of the module.
   * @return module position
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(drivePosition, getMeasuredState().angle);
  }

  /**
   * Calculates the velocity of the wheel from the power applied to the motor.
   * @param power power applied to the motor (-1.0, 1.0)
   * @return velocity of the wheel in meters per second
   */
  public static double calcWheelVelocity(double power) {
    return power * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY;
  }
  
  /**
   * Gets the field-relative pose of the module.
   * @param robotPose pose of the robot on the field
   * @return field-relative pose of the module
   */
  public Pose2d getPose(Pose2d robotPose) {
    Pose2d relativePose = new Pose2d();
    if (corner == 0) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 1) relativePose = new Pose2d(
      SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 2) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    if (corner == 3) relativePose = new Pose2d(
      -SWERVE_DRIVE.WHEELBASE / 2.0,
      -SWERVE_DRIVE.TRACKWIDTH / 2.0,
      getMeasuredState().angle
    );
    return relativePose.relativeTo(new Pose2d(
      new Translation2d(),
      robotPose.getRotation().times(-1.0)
    )).relativeTo( new Pose2d(
      -robotPose.getX(),
      -robotPose.getY(),
      new Rotation2d()
    ));
  }
}