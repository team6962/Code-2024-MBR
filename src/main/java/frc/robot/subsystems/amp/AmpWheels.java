// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.amp;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.util.hardware.SparkMaxUtil;
import frc.robot.util.software.Logging.Logger;



public class AmpWheels extends SubsystemBase {
  private CANSparkMax motor;
  private State state = State.OFF;
  // private NoteDetector detector;
 
  public static enum State {
    IN,
    OUT,
    OFF
  }

  public AmpWheels() {
    motor = new CANSparkMax(CAN.AMP_WHEELS, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, true, CANSparkMax.IdleMode.kBrake);
    SparkMaxUtil.configureCANStatusFrames(motor, false, false);
    SparkMaxUtil.save(motor);
    Logger.autoLog(this, "state", () -> state.name());

    // detector = new NoteDetector(motor, Constants.AMP_WHEELS.GEARING, Constants.AMP_WHEELS.FREE_TORQUE, true);
  }

  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
    if (RobotState.isDisabled()) {
      state = State.OFF;
    }

    switch(state) {
      case OFF:
        motor.set(0);
        break;
      case IN:
        motor.set(-Preferences.AMP_WHEELS.POWER);
        break;
      case OUT:
        motor.set(Preferences.AMP_WHEELS.POWER);
        break;
    }

    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.AMP) motor.stopMotor();
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
