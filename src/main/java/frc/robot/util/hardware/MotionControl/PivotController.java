package frc.robot.util.hardware.MotionControl;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

/*
 * Uses oboard 1kHz PID, Feedforward, and Trapazoidal Profiles to
 * control a pivot mechanism precisely, smoothly, and accurately
 */

public class PivotController {
  private Rotation2d targetAngle = null;
  // State setpointState;
  private double kS = 0.0;
  // Onboard spark max PID controller. Runs at 1kHz
  // private SparkPIDController pid;
  private SparkPIDController pid;
  // CAN Spark Max motor controller;
  private CANSparkMax motor;
  // Built-in relative NEO encoder
  private RelativeEncoder encoder;
  // Rev absolute through-bore encoder
  private DutyCycleEncoder absoluteEncoder;

  private Rotation2d minAngle, maxAngle, tolerance;

  private double encoderOffset = 0.0;

  private boolean reversed;

  private SubsystemBase subsystem;

  private Debouncer debouncer = new Debouncer(0.1);

  private Rotation2d achievableAngle = new Rotation2d();

  private SingleJointedArmSim sim;

  private PIDController simPID;


  public PivotController(SubsystemBase subsystem, CANSparkMax motor, int absoluteEncoderDIO, double absolutePositionOffset, double kP, double kS, double gearing, Rotation2d minAngle, Rotation2d maxAngle, Rotation2d tolerance, boolean reversed) {
    // feedforward = new ArmFeedforward(kS, 0.0, 0.0, 0.0);
    // profile = new TrapezoidProfile(
    //   new Constraints(maxVelocity, maxAcceleration)
    // );
    this.kS = kS;
    pid = motor.getPIDController();
    encoder = motor.getEncoder();
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderDIO);

    this.motor = motor;
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
    this.tolerance = tolerance;

    this.reversed = reversed;
    this.subsystem = subsystem;
    encoderOffset = absolutePositionOffset;

    SparkMaxUtil.configureEncoder(motor, 2.0 * Math.PI / gearing);
    SparkMaxUtil.configurePID(subsystem, motor, kP, 0.0, 0.0, 0.0, false);
    
    Logger.autoLog(subsystem, "targetPosition",                   () -> getTargetAngle().getRadians());
    Logger.autoLog(subsystem, "position",                         () -> getPosition().getRadians());
    Logger.autoLog(subsystem, "relativePosition",                 () -> encoder.getPosition());
    Logger.autoLog(subsystem, "rawAbsolutePosition",              () -> absoluteEncoder.getAbsolutePosition() * 360);
    Logger.autoLog(subsystem, "doneMoving",                       () -> doneMoving());

    StatusChecks.addCheck(subsystem, "absoluteEncoderConnected", () -> absoluteEncoder.isConnected());
    StatusChecks.addCheck(subsystem, "absoluteEncoderUpdated",   () -> absoluteEncoder.getAbsolutePosition() != 0.0);

    sim = new SingleJointedArmSim(
      DCMotor.getNEO(1),
      gearing, 
      SingleJointedArmSim.estimateMOI(1.0, 5.0), 
      1.0,
      Units.degreesToRadians(-180.0), 
      Units.degreesToRadians(180.0),
      false,
      0.0
    );
    simPID = new PIDController(kP, 0, 0);
  }

  public void run() {
    if (targetAngle == null) return; // If we havent set a target angle yet, do nothing
    if (!absoluteEncoder.isConnected()) {
      motor.stopMotor();
      return;
    }

    // System.out.println(getPosition().getDegrees());

    // Re-seed the relative encoder with the absolute encoder when not moving
    // if (doneMoving()) {
    encoder.setPosition(getPosition().getRadians());
    // }
    
    // if (setpointState == null) {
    //   setpointState = new State(getAbsolutePosition().getRadians(), getVelocity().getRadians());
    // };

    // Calculate the setpoint following a trapazoidal profile (smooth ramp up and down acceleration curves)
    // State targetState = new State(targetAngle.getRadians(), 0.0);


    // setpointState = profile.calculate(Robot.getLoopTime(), setpointState, targetState);
    setAchievableAngle();

    if (doneMoving()) {
      motor.stopMotor();
      if (RobotBase.isSimulation()) sim.setInputVoltage(0.0);
      return;
    }

    // Set onboard PID controller to follow
    pid.setReference(
      achievableAngle.getRadians(),
      CANSparkMax.ControlType.kPosition,
      0,
      kS * Math.signum(achievableAngle.getRadians() - getPosition().getRadians())
    );

    if (RobotBase.isSimulation()) sim.setInputVoltage(simPID.calculate(sim.getAngleRads(), achievableAngle.getRadians()) * 12.0);

    // System.out.println(Math.signum(targetAngle.getRadians() - getPosition().getRadians()));
    // System.out.println("kS: " + kS);
    // System.out.println(feedforward.calculate(setpointState.position, setpointState.velocity));

    if (motor.getAppliedOutput() > 0.0 && getPosition().getRadians() > maxAngle.getRadians()) {
      motor.stopMotor();
      if (RobotBase.isSimulation()) sim.setInputVoltage(0.0);
    }

    if (motor.getAppliedOutput() < 0.0 && getPosition().getRadians() < minAngle.getRadians()) {
      motor.stopMotor();
      if (RobotBase.isSimulation()) sim.setInputVoltage(0.0);
    }

    if (Robot.isSimulation()) sim.update(Robot.getLoopTime());
  }

  public void setTargetAngle(Rotation2d angle) {
    targetAngle = angle;
    setAchievableAngle();
  }

  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  public boolean isPastLimit() {
    return encoder.getPosition() > maxAngle.getRadians() || encoder.getPosition() < minAngle.getRadians();
  }

  private void setAchievableAngle() {
    achievableAngle = targetAngle;
    if (achievableAngle.getRadians() < minAngle.getRadians()) {
        achievableAngle = minAngle;
    } else if (achievableAngle.getRadians() > maxAngle.getRadians()) {
        achievableAngle = maxAngle;
    }
  }

  public boolean isAngleAchievable(Rotation2d angle) {
    return angle.getRadians() > minAngle.getRadians() && angle.getRadians() < maxAngle.getRadians();
  }

  public Rotation2d getPosition() {
    if (Robot.isSimulation()) return Rotation2d.fromRadians(sim.getAngleRads());

    double factor = 1;
    if (reversed) {
      factor = -1;
    }

    // ((0.26934 + x) * -1)

    // Map absolute encoder position from 0 - 1 rotations to -pi - pi radians, where 0 is straight out
    double absoluteAngle = (absoluteEncoder.getAbsolutePosition() + encoderOffset) * factor;
    while (absoluteAngle < 0) absoluteAngle++;
    absoluteAngle %= 1.0;
    
    absoluteAngle *= Math.PI * 2.0;
    if (absoluteAngle > Math.PI) {
      absoluteAngle -= Math.PI * 2.0;
    }

    return Rotation2d.fromRadians(absoluteAngle);
  }

  public boolean doneMoving() {
    if (getTargetAngle() == null) return true;
    return debouncer.calculate(Math.abs(getPosition().getRadians() - achievableAngle.getRadians()) < tolerance.getRadians());
  }
  
  public void setMaxAngle(Rotation2d newMaxAngle) {
    maxAngle = newMaxAngle;
  }
}
