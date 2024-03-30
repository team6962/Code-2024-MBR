// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.DIO;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.SHOOTER_PIVOT;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.hardware.MotionControl.PivotController;

public class ShooterPivot extends SubsystemBase {
  private CANSparkMax motor;
  private PivotController controller;
  private boolean isCalibrating = false;

  public ShooterPivot() {    
    motor = new CANSparkMax(CAN.SHOOTER_PIVOT, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, false, CANSparkMax.IdleMode.kBrake); // TODO CHANGE TO BRAKE
    // SparkMaxUtil.save(motor);
    // SparkMaxUtil.configureCANStatusFrames(motor, false, true);

    controller = new PivotController(
      this,
      motor,
      DIO.SHOOTER_PIVOT,
      SHOOTER_PIVOT.ABSOLUTE_POSITION_OFFSET,
      SHOOTER_PIVOT.PROFILE.kP,
      SHOOTER_PIVOT.PROFILE.kS,
      SHOOTER_PIVOT.GEARING,
      Preferences.SHOOTER_PIVOT.MIN_ANGLE,
      Preferences.SHOOTER_PIVOT.MAX_ANGLE,
      Rotation2d.fromDegrees(0.25),
      true
    );

    SparkMaxUtil.save(motor);

    // new TunableNumber(this, "Shooter Pivot Power", (x) -> controller.setTargetAngle(Rotation2d.fromDegrees(x)), 0);
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    if (isCalibrating) return;
    if (RobotState.isDisabled()) {
      controller.setTargetAngle(controller.getPosition());
    }

    controller.run();

    // motor.set(0.33);
    
    if (motor.getAppliedOutput() > 0.0 && getPosition().getRadians() > Preferences.SHOOTER_PIVOT.MAX_ANGLE.getRadians()) {
      motor.stopMotor();
    }

    if (motor.getAppliedOutput() < 0.0 && getPosition().getRadians() < Preferences.SHOOTER_PIVOT.MIN_ANGLE.getRadians()) {
      motor.stopMotor();
    }

    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.SHOOTER) motor.stopMotor();
  }

  public Command setTargetAngleCommand(Supplier<Rotation2d> angleSupplier) {
    return runEnd(
      () -> setTargetAngle(angleSupplier.get()),
      () -> setTargetAngle(Preferences.SHOOTER_PIVOT.IDLE_ANGLE)
    );
  }

  public boolean isAngleAchievable(Rotation2d angle) {
    if (angle == null) return false;
    return controller.isAngleAchievable(angle);
  }

  public void setTargetAngle(Rotation2d angle) {
    if (angle == null) return;
    controller.setTargetAngle(angle);
  }

  public Rotation2d getTargetAngle() {
    return controller.getTargetAngle();
  }

  public Rotation2d getPosition() {
    return controller.getPosition();
  }

  public boolean doneMoving() {
    return controller.doneMoving();
  }

  public void setMaxAngle(Rotation2d angle) {
    controller.setMaxAngle(angle);
  }

  // public Command calibrate() {
  //   SysIdRoutine calibrationRoutine = new SysIdRoutine(
  //     new SysIdRoutine.Config(),
  //     new SysIdRoutine.Mechanism(
  //       (Measure<Voltage> volts) -> {
  //         motor.setVoltage(volts.in(Volts));
  //       },
  //       log -> {
  //         log.motor("shooter-pivot")
  //           .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
  //           .angularPosition(Radians.of(controller.getPosition().getRadians()))
  //           .angularVelocity(RadiansPerSecond.of(controller.getVelocity().getRadians()));
  //       },
  //       this
  //     )
  //   );

  //   return Commands.sequence(
  //     Commands.runOnce(() -> isCalibrating = true),
  //     calibrationRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(controller::isPastLimit),
  //     Commands.runOnce(() -> motor.stopMotor()),
  //     Commands.waitSeconds(1.0),
  //     calibrationRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(controller::isPastLimit),
  //     Commands.runOnce(() -> motor.stopMotor()),
  //     Commands.waitSeconds(1.0),
  //     calibrationRoutine.dynamic(SysIdRoutine.Direction.kForward).until(controller::isPastLimit),
  //     Commands.runOnce(() -> motor.stopMotor()),
  //     Commands.waitSeconds(1.0),
  //     calibrationRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(controller::isPastLimit),
  //     Commands.runOnce(() -> motor.stopMotor()),
  //     Commands.waitSeconds(1.0),
  //     Commands.runOnce(() -> isCalibrating = false)
  //   );
  // }
}
