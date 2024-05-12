// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.Constants.Field;
import frc.robot.commands.autonomous.Autonomous;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.vision.AprilTags;
import frc.robot.util.software.CustomSwerveDrivePoseEstimator;
import frc.robot.util.software.MathUtils;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

/**
 * This class represents the subsystem for the swerve drive. It contains four
 * swerve module objects and a gyroscope object.
 */
public class SwerveDrive extends SubsystemBase {

  // Swerve kinematics
  public SwerveModule[] modules = new SwerveModule[SWERVE_DRIVE.MODULE_COUNT];
  private SwerveDriveKinematics kinematics = getKinematics();
  private CustomSwerveDrivePoseEstimator poseEstimator;
  private static Field2d field = new Field2d(); // Field2d object for SmartDashboard widget
  private ChassisSpeeds drivenChassisSpeeds = new ChassisSpeeds();
  private SWERVE_DRIVE.MODULE_CONFIG[] equippedModules;
  private SwerveDriveWheelPositions previousWheelPositions;
  private Translation2d linearAcceleration;

  // Gyro & rotation
  private static AHRS gyro; // NAVX2-MXP Gyroscope
  private Rotation2d gyroHeading = Rotation2d.fromDegrees(0.0);
  private Rotation2d gyroOffset = SWERVE_DRIVE.STARTING_POSE.get().getRotation();
  private Debouncer doneRotating = new Debouncer(0.5);
  private double addedAlignmentAngularVelocity = 0.0;
  private double angularAcceleration = 0.0;
  private PIDController alignmentController = new PIDController(
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kP,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kI,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kD
  );

  // Autonomous
  private Supplier<Translation2d> rotationOverridePoint = null;
  private Rotation2d rotationOverrideOffset = new Rotation2d();

  // States
  private boolean isAligning = false;
  private boolean parked = false;
  private boolean parkingDisabled = false;
  private boolean isDriven = false;
  private boolean gyroConnected = false;
  
  public SwerveDrive() {
    // Create the serve module objects
    equippedModules = SWERVE_DRIVE.IS_PROTOTYPE_CHASSIS ? SWERVE_DRIVE.EQUIPPED_MODULES_PROTOTYPE : SWERVE_DRIVE.EQUIPPED_MODULES_COMPETITION;
    int corner = 0;
    for (SWERVE_DRIVE.MODULE_CONFIG config : equippedModules) {
      String name = SWERVE_DRIVE.MODULE_NAMES[corner];
      if (RobotBase.isSimulation()) { // Use sim modules in simulation
        modules[corner] = new SwerveModuleSim(config, corner, name);
      } else {
        modules[corner] = new SwerveModule(config, corner, name);
      }
      corner++;
    }

    // Set up pose estimator and rotation controller
    poseEstimator = new CustomSwerveDrivePoseEstimator(
      kinematics,
      SWERVE_DRIVE.STARTING_POSE.get().getRotation(),
      getModulePositions().positions,
      SWERVE_DRIVE.STARTING_POSE.get(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2)),
      VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(30))
    );

    previousWheelPositions = getModulePositions(); // Used for twist calculations for odometry
    alignmentController.enableContinuousInput(-Math.PI, Math.PI); // Set the controller to be continuous
    alignmentController.setTolerance(SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.TOLERANCE.getRadians());
    
    // If possible, connect to the gyroscope
    try {
      gyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), false);
    }

    // Gyro takes a little while to calibrate, so we wait a second before setting the offset
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyroOffset = gyroOffset.minus(gyro.getRotation2d());
      } catch (Exception e) {}
    }).start();
    
    
    // Log data
    SmartDashboard.putData("Field", field);
    Logger.autoLog(this, "pose", () -> this.getPose());
    Logger.autoLog(this, "measuredHeading", () -> this.getHeading().getDegrees());
    Logger.autoLog(this, "targetHeading", () -> Units.radiansToDegrees(alignmentController.getSetpoint()));
    Logger.autoLog(this, "targetStates", () -> getTargetModuleStates());
    Logger.autoLog(this, "measuredStates", () -> getMeasuredModuleStates());
    Logger.autoLog(this, "modulePositions", () -> getModulePositions());
    Logger.autoLog(this, "gyroAcceleration", () -> Math.hypot(gyro.getWorldLinearAccelX(), gyro.getWorldLinearAccelY()));
    Logger.autoLog(this, "gyroVelocity", () -> Math.hypot(gyro.getVelocityX(), gyro.getVelocityY()));
    Logger.autoLog(this, "commandedLinearAcceleration", () -> linearAcceleration.getNorm());
    Logger.autoLog(this, "commandedLinearVelocity", () -> Math.hypot(getDrivenChassisSpeeds().vxMetersPerSecond, getDrivenChassisSpeeds().vyMetersPerSecond));
    Logger.autoLog(this, "commandedAngularAcceleration", () -> angularAcceleration);
    Logger.autoLog(this, "commandedAngularVelocity", () -> getDrivenChassisSpeeds().omegaRadiansPerSecond);
    Logger.autoLog(this, "measuredAngularVelocity", () -> getMeasuredChassisSpeeds().omegaRadiansPerSecond);
    Logger.autoLog(this, "measuredLinearVelocity", () -> Math.hypot(getMeasuredChassisSpeeds().vxMetersPerSecond, getMeasuredChassisSpeeds().vyMetersPerSecond));
    Logger.autoLog(this, "gyroIsCalibrating", () -> gyro.isCalibrating());
    Logger.autoLog(this, "gyroIsConnected", () -> gyro.isConnected());
    Logger.autoLog(this, "gyroRawDegrees", () -> gyro.getRotation2d().getDegrees());
    StatusChecks.addCheck(this, "isGyroConnected", gyro::isConnected);

    // Path planner setup
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getMeasuredChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD), // Translation PID constants
        new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kD), // Rotation PID constants
        SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY, // Max module speed, in m/s
        SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
      ),
      () -> false,
      this // Reference to this subsystem to set requirements
    );

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        field.getObject("Target Pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        // Do whatever you want with the poses here
        field.getObject("Active Path").setPoses(poses);
    });
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_DRIVE) return;
    // If the robot is disabled, reset states and target heading
    if (RobotState.isDisabled()) {
      for (SwerveModule module : modules) {
        module.seedSteerEncoder();
      }
      setTargetHeading(getHeading());
      isAligning = false;
      rotationOverridePoint = null;
    }
    
    updateOdometry();

    // Update field
    FieldObject2d modulesObject = field.getObject("Swerve Modules");
    Pose2d[] modulePoses = new Pose2d[SWERVE_DRIVE.MODULE_COUNT];
    Pose2d robotPose = getPose();
    int i = 0;
    for (SwerveModule module : modules) {
      modulePoses[i] = module.getPose(robotPose);
      i++;
    }
    modulesObject.setPoses(modulePoses);
    field.setRobotPose(getPose());
    
    // If drive calls are not being made, stop the robot
    if (!isDriven) driveFieldRelative(0.0, 0.0, 0.0);
    isDriven = false;
  }

  public void updateOdometry() {
    Pose2d poseBefore = getPose();

    // Math to use the swerve modules to calculate the robot's rotation if the gyro disconnects
    SwerveDriveWheelPositions wheelPositions = getModulePositions();
    Twist2d twist = kinematics.toTwist2d(previousWheelPositions, wheelPositions);
    Pose2d newPose = getPose().exp(twist);
    if (!gyroConnected && (gyro.isConnected() && !gyro.isCalibrating())) {
      gyroOffset = gyroHeading.minus(gyro.getRotation2d());
    }
    gyroConnected = gyro.isConnected() && !gyro.isCalibrating();
    if (gyroConnected && !RobotBase.isSimulation()) {
      gyroHeading = gyro.getRotation2d();
    } else {
      gyroHeading = gyroHeading.plus(newPose.getRotation().minus(getPose().getRotation()));
    }

    poseEstimator.update(gyroHeading.plus(gyroOffset), getModulePositions());
    AprilTags.injectVisionData(LIMELIGHT.APRILTAG_CAMERA_POSES, this); // Limelight vision data from apriltags

    // Sometimes the vision data will cause the robot to go crazy, so we check if the robot is moving too fast and reset the pose if it is
    Pose2d currentPose = getPose();
    double magnitude = currentPose.getTranslation().getNorm();
    if (magnitude > 1000 || Double.isNaN(magnitude) || Double.isInfinite(magnitude)) {
      System.out.println("BAD");
      LEDs.setState(LEDs.State.BAD);
      resetPose(gyroHeading.plus(gyroOffset), poseBefore, previousWheelPositions);
    }
    
    previousWheelPositions = wheelPositions.copy();
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Drives the robot at a given field-relative velocity
   * @param xVelocity [meters / second] Positive x is away from your alliance wall
   * @param yVelocity [meters / second] Positive y is to your left when standing behind the alliance wall
   * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
   */
  public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  /**
   * 
   * Drives the robot at a given field-relative ChassisSpeeds
   * @param fieldRelativeSpeeds
   */
  private void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveAttainableSpeeds(fieldRelativeSpeeds);
  }

  /**
   * Drives the robot at a given robot-relative velocity
   * @param xVelocity [meters / second] Positive x is towards the robot's front
   * @param yVelocity [meters / second] Positive y is towards the robot's left
   * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
   */
  public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveRobotRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  /**
   * Drives the robot at a given robot-relative ChassisSpeeds
   * @param robotRelativeSpeeds
   */
  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getAllianceAwareHeading()));
  }

  /**
   * Drives the robot at a given field-relative ChassisSpeeds. This is the base method, and all other drive methods call this one.
   * @param fieldRelativeSpeeds The desired field-relative speeds
   */
  private void driveAttainableSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
    isDriven = true;

    // Avoid obstacles
    if (!(RobotState.isAutonomous() && !Autonomous.avoidPillars)) {
      Translation2d velocity = XBoxSwerve.avoidObstacles(new Translation2d(
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond
      ), this);
      fieldRelativeSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), fieldRelativeSpeeds.omegaRadiansPerSecond);
    }

    // If the robot is rotating, cancel the rotation override
    if (fieldRelativeSpeeds.omegaRadiansPerSecond > 0 && !RobotState.isAutonomous()) {
      rotationOverridePoint = null;
    }

    // If the robot is in autonomous mode, or if a rotation override point is set, stop the robot from rotating
    if (rotationOverridePoint != null || RobotState.isAutonomous()) {
      fieldRelativeSpeeds.omegaRadiansPerSecond = 0.0;
      if (rotationOverridePoint != null) facePoint(rotationOverridePoint.get(), rotationOverrideOffset);
    }
    
    // Conditionals to compensate for the slop in rotation when aligning with PID controllers
    if (Math.abs(fieldRelativeSpeeds.omegaRadiansPerSecond) > 0.01) {
      setTargetHeading(getHeading());
      isAligning = false;
    }
    if (!isAligning && doneRotating.calculate(Math.abs(getDrivenChassisSpeeds().omegaRadiansPerSecond) < 0.1)) {
      setTargetHeading(getHeading());
      isAligning = true;
    }
    
    // Calculate the angular velocity to align with the target heading
    double alignmentAngularVelocity = alignmentController.calculate(getHeading().getRadians()) + addedAlignmentAngularVelocity;
    addedAlignmentAngularVelocity = 0.0;
    if (isAligning && !alignmentController.atSetpoint() && !parked) fieldRelativeSpeeds.omegaRadiansPerSecond += alignmentAngularVelocity;

    // Calculate the wheel speeds to achieve the desired field-relative speeds
    SwerveDriveWheelStates wheelSpeeds = kinematics.toWheelSpeeds(fieldRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      wheelSpeeds.states,
      fieldRelativeSpeeds,
      SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
      SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
      SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY
    );
    fieldRelativeSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

    // Limit translational acceleration
    Translation2d targetLinearVelocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    Translation2d currentLinearVelocity = new Translation2d(drivenChassisSpeeds.vxMetersPerSecond, drivenChassisSpeeds.vyMetersPerSecond);
    linearAcceleration = (targetLinearVelocity).minus(currentLinearVelocity).div(Robot.getLoopTime());
    double linearForce = linearAcceleration.getNorm() * SWERVE_DRIVE.ROBOT_MASS;

    // Limit rotational acceleration
    double targetAngularVelocity = fieldRelativeSpeeds.omegaRadiansPerSecond;
    double currentAngularVelocity = drivenChassisSpeeds.omegaRadiansPerSecond;
    angularAcceleration = (targetAngularVelocity - currentAngularVelocity) / Robot.getLoopTime();
    double angularForce = Math.abs((SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA * angularAcceleration) / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS);
    
    // Limit the total force applied to the robot
    double frictionForce = 9.80 * SWERVE_DRIVE.ROBOT_MASS * SWERVE_DRIVE.FRICTION_COEFFICIENT;
    if (linearForce + angularForce > frictionForce) {
      double factor = (linearForce + angularForce) / frictionForce;
      linearAcceleration = linearAcceleration.div(factor);
      angularAcceleration /= factor;
    }

    // Calculate the attainable linear and angular velocities and update the driven chassis speeds
    Translation2d attainableLinearVelocity = currentLinearVelocity.plus(linearAcceleration.times(Robot.getLoopTime()));
    double attainableAngularVelocity = currentAngularVelocity + (angularAcceleration * Robot.getLoopTime());
    drivenChassisSpeeds = new ChassisSpeeds(attainableLinearVelocity.getX(), attainableLinearVelocity.getY(), attainableAngularVelocity);
    
    // Discretize converts a continous-time chassis speed into discrete-time to compensate for the loop time. This helps reduce drift when rotating while driving in a straight line.
    drivenChassisSpeeds = ChassisSpeeds.discretize(drivenChassisSpeeds, Robot.getLoopTime());
    SwerveDriveWheelStates drivenModuleStates = kinematics.toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(drivenChassisSpeeds, getAllianceAwareHeading()));
    
    // If the robot is not moving, park the modules
    boolean moving = false;
    for (SwerveModuleState moduleState : kinematics.toWheelSpeeds(fieldRelativeSpeeds).states) if (Math.abs(moduleState.speedMetersPerSecond) > 0.0) moving = true;
    for (SwerveModuleState moduleState : drivenModuleStates.states) if (Math.abs(moduleState.speedMetersPerSecond) > 0.0) moving = true;
    parked = false;

    if (!moving) {
      if (!parkingDisabled) {
        parkModules();
        return;
      }
      // If the robot is aligning, park the modules in a different configuration.
      // Parking normally while aligning can cause the robot to drift away from the setpoint.
      if (alignmentController.atSetpoint()) {
        parkForAlignment();
      }
    }

    parkingDisabled = false;
    driveModules(drivenModuleStates);
  }
  
  /**
   * Drives the swerve modules at the calculated speeds
   * @param wheelSpeeds The calculated speeds and directions for each module
   */
  private void driveModules(SwerveDriveWheelStates wheelSpeeds) {
    // Drive the swerve modules at the calculated speeds
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      modules[i].setTargetState(wheelSpeeds.states[i]);
    }
  }

  /**
   * Sets the target heading for the robot
   * @param heading The target heading for the robot
   */
  public void setTargetHeading(Rotation2d heading) {
    alignmentController.setSetpoint(heading.getRadians());
    parkingDisabled = true;
  }

  /**
   * Sets the target heading for the robot
   * @param heading The target heading for the robot
   */
  public void setTargetHeadingAndVelocity(Rotation2d heading, double velocity) {
    setTargetHeading(heading);
    addedAlignmentAngularVelocity = velocity;
  }

  /**
   * Faces a point on the field
   * @param point The point on the field we want to face
   */
  public Command facePointCommand(Supplier<Translation2d> point, Rotation2d rotationOffset) {
    return Commands.run(
      () -> facePoint(point.get(), rotationOffset)
    );
  }

  /**
   * Faces a point on the field
   * @param point
   * @param rotationOffset
   */
  public void facePoint(Translation2d point, Rotation2d rotationOffset) {
    double time = 0.02;

    // With no point, do nothing.
    if (point == null) {
      setTargetHeadingAndVelocity(getHeading(), 0.0);
      return;
    }

    // If the robot is close to the point, do nothing.
    if (point.getDistance(getPose().getTranslation()) < 1.0 && RobotState.isAutonomous()) {
      return;
    }

    // Calculate the future position of the robot, and predict how the heading will need to change in the future.
    Translation2d currentPosition = getPose().getTranslation();
    Translation2d futurePosition = getPose().getTranslation().plus(getFieldVelocity().times(time));
    Rotation2d currentTargetHeading = point.minus(currentPosition).getAngle().plus(rotationOffset);
    Rotation2d futureTargetHeading = point.minus(futurePosition).getAngle().plus(rotationOffset);    
    double addedVelocity = futureTargetHeading.minus(currentTargetHeading).getRadians() / time;
    if (getPose().getTranslation().getDistance(point) < 1.0) {
      addedVelocity = 0.0;
    }
    setTargetHeadingAndVelocity(currentTargetHeading, addedVelocity);
  }


  /**
   * 
   * @return The target heading for the robot
   */
  public Rotation2d getTargetHeading() {
    return Rotation2d.fromRadians(alignmentController.getSetpoint());
  }

  /**
   * Sets a rotation point override so the robot always points in that direction. Used for autonomous mostly.
   * @param point The point to face
   * @param rotationOffset The offset to add to the rotation. 180 degrees would point away from it.
   */
  public void setRotationTargetOverrideFromPoint(Supplier<Translation2d> point, Rotation2d rotationOffset) {
    rotationOverridePoint = point;
    rotationOverrideOffset = rotationOffset;
    addedAlignmentAngularVelocity = 0.0;
  }

  /**
   * This creates an "X" pattern with the wheels which makes the robot very hard to move
   */
  private void parkModules() {
    modules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    parked = true;
  }

  /**
   * This creates an "O" pattern with the wheels which makes the robot very hard to translate, but still allows for rotation
   */
  private void parkForAlignment() {
    modules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Position to reset the odometer to
   */
  public void resetPose(Rotation2d heading, Pose2d pose, SwerveDriveWheelPositions wheelPositions) {
    poseEstimator.resetPosition(heading, wheelPositions, pose);
    alignmentController.setSetpoint(getHeading().getRadians());
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Position to reset the odometer to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    alignmentController.setSetpoint(getHeading().getRadians());
  }

  /**
   * Checks if the robot can zero its heading
   * @return
   */
  public boolean canZeroHeading() {
    return (parked || isAligning || RobotState.isDisabled()) && (Math.abs(getRotationalVelocity()) < 0.5);
  }

  /**
   * 
   * @param visionMeasurement The robot position on the field from the apriltags
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) {
    Rotation2d oldHeading = getHeading();
    poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    poseEstimator.addVisionMeasurement(visionMeasurement, timestamp);
    Rotation2d newHeading = getHeading();
    alignmentController.setSetpoint(Rotation2d.fromRadians(alignmentController.getSetpoint()).plus(newHeading).minus(oldHeading).getRadians());
  }

  /**
   * Stops all motors on all modules
   */
  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /**
   * Returns the field-relative velocity of the robot
   * @return Field-relative velocity in m/s
   */
  public Translation2d getFieldVelocity() {
    ChassisSpeeds fieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeeds(), getHeading());
    return new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond);
  }

  /**
   * Returns the rotational velocity of the robot
   * @return Rotational velocity in rad/s
   */
  public double getRotationalVelocity() {
    return getMeasuredChassisSpeeds().omegaRadiansPerSecond;
  }

  /**
   * @return Target chassis x, y, and rotational velocity (robot-relative)
   */
  private ChassisSpeeds getTargetChassisSpeeds() {
    return kinematics.toChassisSpeeds(getTargetModuleStates());
  }

  /**
   * @return Measured chassis x velocity, y velocity, and rotational velocity (robot-relative)
   */
  private ChassisSpeeds getMeasuredChassisSpeeds() {
    return kinematics.toChassisSpeeds(getMeasuredModuleStates());
  }

  /**
   * @return Driven chassis x speed, y speed, and rotational speed (robot-relative)
   */
  private ChassisSpeeds getDrivenChassisSpeeds() {
    return drivenChassisSpeeds;
  }

  /**
   * @return Measured module positions
   */
  public SwerveDriveWheelPositions getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    return new SwerveDriveWheelPositions(positions);
  }

  /**
   * @return Target module states (speed and direction)
   */
  private SwerveDriveWheelStates getTargetModuleStates() {
    SwerveModuleState[] targetStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      targetStates[i] = modules[i].getTargetState();
    }
    return new SwerveDriveWheelStates(targetStates);
  }

  /**
   * @return Measured module states (speed and direction)
   */
  private SwerveDriveWheelStates getMeasuredModuleStates() {
    SwerveModuleState[] measuredStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      measuredStates[i] = modules[i].getMeasuredState();
    }
    return new SwerveDriveWheelStates(measuredStates);
  }

  /**
   * @return This swerve drive's NavX AHRS IMU Gyro
   */
  public static AHRS getGyro() {
    return gyro;
  }

  /**
   * Resets gyro heading
   */
  public void resetGyroHeading(Rotation2d newHeading) {
    gyroOffset = newHeading.minus(gyroHeading);
    alignmentController.reset();
    alignmentController.setSetpoint(newHeading.getRadians());
  }

  /**
   * @return Gyro heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * @return Heading as a Rotation2d based on alliance
   */
  public Rotation2d getAllianceAwareHeading() {
    return getHeading().plus(Rotation2d.fromDegrees(Constants.IS_BLUE_TEAM.get() ? 0.0 : 180.0));
  }

  /**
   * @return Pose on the field from odometer data as a Pose2d
   */
  public Pose2d getPose() {
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
    return estimatedPose;
  }

  /**
   * Get the pose of the robot at a given timestamp in the past (up to 1 or 2 seconds ago)
   * @param timestampSeconds The timestamp to get the pose at (Timer.getFPGATimestamp() is the current timestamp)
   * @return Pose on the field from odometer data as a Pose2d
   */
  public Pose2d getPose(double timestampSeconds) {
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition(timestampSeconds);
    return estimatedPose;
  }

  /**
   * Predicts the future pose of the robot based on the current velocity. This pose is if the robot suddenly decelerates to 0 m/s.
   * @return Future pose on the field
   */
  public Pose2d getFuturePose() {
    Translation2d futurePosition = getPose().getTranslation();
    futurePosition = futurePosition.plus(getFieldVelocity().times(getFieldVelocity().getNorm()).div(2.0 * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION));
    return new Pose2d(futurePosition, getPose().getRotation());
  }

  /**
   * Is the robot under the stage?
   * @return
   */
  public boolean underStage() {
    if (!RobotState.isAutonomous()) {
      return MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], getFuturePose().getTranslation()) ||
             MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], getFuturePose().getTranslation()) ||
             MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], getPose().getTranslation()) ||
             MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], getPose().getTranslation());
    } else {
      return MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], getFuturePose().getTranslation()) ||
             MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], getFuturePose().getTranslation());
    }
  }

  /**
   * @return Field2d object for SmartDashboard widget.
   */
  public static Field2d getField() {
    return field;
  }

  /**
   * Converts the speed of a wheel moving to the angular velocity of the robot as if it's
   * rotating in place
   * @param wheelSpeed Drive velocity in m/s
   * @return Equivalent rotational velocity in rad/s
   * @see #toLinear(double)
   */
  public static double toAngular(double wheelSpeed) {
    return wheelSpeed / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }

  /**
   * Converts the angular velocity of the robot to the speed of a wheel moving as if the
   * robot is rotating in place
   * @param angularVelocity Rotational velocity in rad/s
   * @return Equivalent drive velocity in m/s
   * @see #toAngular(double)
   */
  public static double toLinear(double angularVelocity) {
    return angularVelocity * SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }

  /**
   * Create a kinematics object for the swerve drive based on the SWERVE_DRIVE constants
   * @return A SwerveDriveKinematics object that models the swerve drive
  */
  public static SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0));
  }

  /**
   * Pathfind to a first given point on the field, and then follow a straight line to the second point.
   * @param firstPoint
   * @param secondPoint
   * @return
   */
  public Command pathfindThenFollowPath(Pose2d firstPoint, Pose2d secondPoint) {
    Rotation2d angle = secondPoint.getTranslation().minus(firstPoint.getTranslation()).getAngle();

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(firstPoint.getTranslation(), angle),
      new Pose2d(secondPoint.getTranslation(), angle)
    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      new GoalEndState(
        0.0,
        secondPoint.getRotation(),
        true
      )
    );
    
    return AutoBuilder.pathfindThenFollowPath(
      path,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  /**
   * Go to a position on the field (without object avoidence)
   * @param pose Field-relative pose on the field to go to
   * @param xboxController Xbox controller to cancel the command
   * @return A command to run
   */
  public Command goToSimple(Pose2d pose) {
    Rotation2d angle = pose.getTranslation().minus(getPose().getTranslation()).getAngle();

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(getPose().getTranslation(), angle),
      new Pose2d(pose.getTranslation(), angle)
    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      new GoalEndState(
        0.0,
        pose.getRotation(),
        true
      )
    );

    return Commands.sequence(
      AutoBuilder.followPath(path),
      runOnce(() -> setTargetHeading(pose.getRotation()))
    );
  }
}