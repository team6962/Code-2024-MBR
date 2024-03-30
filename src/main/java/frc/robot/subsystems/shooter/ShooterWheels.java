// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.SHOOTER_WHEELS;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.Logging.Logger;

public class ShooterWheels extends SubsystemBase {
  private CANSparkMax shooterMotor, shooterMotorFollower, feedMotor;
  private RelativeEncoder encoder;
  private boolean isCalibrating = false;
  private State state = State.OFF;
  private double speed = ShooterMath.calcShooterWheelVelocity(Constants.SHOOTER_WHEELS.TOP_EXIT_VELOCITY);
  private double encoderVelocity = 0.0;
  private boolean isShooting = false;
  private Debouncer isShootingDebouncer = new Debouncer(2.0, DebounceType.kFalling);
  
  public enum State {
    SPIN_UP,
    REVERSE,
    OFF,
  }

  public ShooterWheels() {
    shooterMotor = new CANSparkMax(CAN.SHOOTER_WHEELS_BOTTOM, MotorType.kBrushless);
    shooterMotorFollower = new CANSparkMax(CAN.SHOOTER_WHEELS_TOP, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, shooterMotorFollower, false, IdleMode.kCoast);
    
    encoder = shooterMotor.getEncoder();

    SparkMaxUtil.configureAndLog(this, shooterMotor, false, CANSparkMax.IdleMode.kCoast);
    SparkMaxUtil.configureEncoder(shooterMotor, SHOOTER_WHEELS.ENCODER_CONVERSION_FACTOR);
    SparkMaxUtil.configureEncoder(shooterMotorFollower, SHOOTER_WHEELS.ENCODER_CONVERSION_FACTOR);
    // SparkMaxUtil.configurePID(this, motor, SHOOTER_WHEELS.PROFILE.kP, SHOOTER_WHEELS.PROFILE.kI, SHOOTER_WHEELS.PROFILE.kD, SHOOTER_WHEELS.PROFILE.kV, false);
    SparkMaxUtil.save(shooterMotor);
    SparkMaxUtil.configureCANStatusFrames(shooterMotor, true, false);
    SparkMaxUtil.configureCANStatusFrames(shooterMotorFollower, true, false);

    shooterMotorFollower.follow(shooterMotor, true);
    SparkMaxUtil.save(shooterMotorFollower);

    feedMotor = new CANSparkMax(CAN.SHOOTER_FEED, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, feedMotor, true, CANSparkMax.IdleMode.kCoast, 80, 80);
    SparkMaxUtil.save(feedMotor);
    SparkMaxUtil.configureCANStatusFrames(feedMotor, false, false);

    Logger.autoLog(this, "velocity", () -> getVelocity());
    Logger.autoLog(this, "targetVelocity", () -> speed);
    Logger.autoLog(this, "state", () -> state.name());
  }

  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }

  public double getVelocity() {
    if (Robot.isSimulation()) return state == State.SPIN_UP ? speed : 0.0;
    return Math.round(encoderVelocity / 10.0) * 10.0;
  }

  public State getState() {
    return state;
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;

    encoderVelocity = encoder.getVelocity();

    if (isCalibrating) return;
    
    if (RobotState.isDisabled()) {
      state = State.OFF;
    }

    if (RobotState.isAutonomous()) {
      state = State.SPIN_UP;
      speed = ShooterMath.calcShooterWheelVelocity(Constants.SHOOTER_WHEELS.TOP_EXIT_VELOCITY);
    }

    // System.out.println(speed);
    // System.out.println(ShooterMath.calcProjectileVelocity(ShooterMath.calcShooterWheelVelocity(speed)));
    double motorSpeed;
    switch(state) {
      case SPIN_UP:
        // System.out.println(speed);
        motorSpeed = (speed / SHOOTER_WHEELS.MAX_WHEEL_SPEED);
        shooterMotor.set(motorSpeed / 0.8888349515 / 1.0328467153 / 0.975257732);
        if (isShootingDebouncer.calculate(isShooting)) {
          feedMotor.set(motorSpeed * (62.0 / 38.0) / 1.125);
        } else {
          feedMotor.set(0.0);
        }
        break;
      case REVERSE:
        shooterMotor.set(-1.0);
        feedMotor.set(-1.0);
        break;
      case OFF:
        shooterMotor.set(0.0);
        feedMotor.set(0.0);
        break;
    }

    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SHOOTER) {
      shooterMotor.stopMotor();
      shooterMotorFollower.stopMotor();
    }
  }

  public Command setTargetWheelSpeedCommand(Supplier<Double> speed) {
    return Commands.runEnd(
      () -> this.speed = speed.get(),
      () -> this.speed = ShooterMath.calcShooterWheelVelocity(Constants.SHOOTER_WHEELS.TOP_EXIT_VELOCITY)
    );
  }

  public Command setTargetExitVelocityCommand(Supplier<Double> exitVelocity) {
    return Commands.runEnd(
      () -> this.speed = ShooterMath.calcShooterWheelVelocity(Math.round(exitVelocity.get() * 10.0) / 10.0),
      () -> this.speed = ShooterMath.calcShooterWheelVelocity(Constants.SHOOTER_WHEELS.TOP_EXIT_VELOCITY)
    );
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }

  public double getTargetVelocity() {
    return speed;
  }

  public Command turnOnFeedWheels() {
    return Commands.runEnd(() -> isShooting = true, () -> isShooting = false);
  }

  public Command calibrate() {
    SysIdRoutine calibrationRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> {
          shooterMotor.setVoltage(volts.in(Volts));
        },
        log -> {
          log.motor("shooter-wheels")
            .voltage(Volts.of(shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage()))
            .angularPosition(Radians.of(encoder.getPosition()))
            .angularVelocity(RadiansPerSecond.of(encoder.getVelocity()));
        },
        this
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> isCalibrating = true),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> shooterMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> shooterMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward),
      Commands.runOnce(() -> shooterMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
      Commands.runOnce(() -> shooterMotor.stopMotor()),
      Commands.waitSeconds(1.0),
      Commands.runOnce(() -> isCalibrating = false)
    );
  }
}
