// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hang;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.CAN;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.HANG;
import frc.robot.Constants.Preferences;
import frc.robot.Constants.Preferences.VOLTAGE_LADDER;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.hardware.SparkMaxUtil;

public class Hang extends SubsystemBase {
    private CANSparkMax rightMotor;
    private RelativeEncoder rightEncoder;
    private State state = State.OFF; 
    
    private AHRS gyro; 
    private double gyroRotation; 
    
    public static enum State {
        EXTEND,
        RETRACT,
        OFF
    }
      

  /** Creates a new ExampleSubsystem. */
  public Hang() {
    if (!Constants.ENABLED_SYSTEMS.ENABLE_HANG) return;
    
    gyro = SwerveDrive.getGyro();
    // Gyro = SwerveDrive.getGyro()

    rightMotor = new CANSparkMax(CAN.HANG_RIGHT, MotorType.kBrushless);
    
    SparkMaxUtil.configureAndLog(this, rightMotor, false, CANSparkMax.IdleMode.kBrake);
    SparkMaxUtil.configureCANStatusFrames(rightMotor, false, true);
    SparkMaxUtil.save(rightMotor);   

    rightEncoder = rightMotor.getEncoder();

    SparkMaxUtil.configureEncoder(rightMotor, (1.0 / HANG.GEARING) * 2.0 * Math.PI * HANG.SPOOL_RADIUS);
 
  }
  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_HANG) return;

    if (RobotState.isDisabled()) {
      state = State.OFF;
    }

    double rightMotorPower = 0.0;

    switch(state) {
      case OFF:
        rightMotorPower = 0.0;
        break;
      case EXTEND:
        rightMotorPower = Preferences.HANG.RIGHT_MOTOR_EXTEND_POWER;
        break;
      case RETRACT:
        rightMotorPower = -1.0;

        // // if it's tilting to the left and it can still retract
        // if (gyroRotation > Preferences.HANG.MAX_ROLL_ANGLE) {
        //   // then retract the left side
        //   leftMotorPower = Preferences.HANG.LEFT_MOTOR_RETRACT_POWER;
        // }
        // // if it's tilting to the right and it can still retract
        // else if (gyroRotation < -Preferences.HANG.MAX_ROLL_ANGLE) {
        //   // then retract the right side
        //   rightMotorPower = Preferences.HANG.RIGHT_MOTOR_RETRACT_POWER;
        // }
        // else {
        //   rightMotorPower = Preferences.HANG.RIGHT_MOTOR_RETRACT_POWER;
        //   leftMotorPower = Preferences.HANG.LEFT_MOTOR_RETRACT_POWER;
        // }
        break;
    }

    // System.out.println(rightMotorPower);

    // Makes sure we dont overshoot our limits for right hang arm
    if ((rightEncoder.getPosition() >= Constants.HANG.EXTEND_HEIGHT && rightMotorPower > 0.0) || (rightEncoder.getPosition() <= Constants.HANG.RETRACT_HEIGHT && rightMotorPower < 0.0)) {
      rightMotorPower = 0.0;
    }

    // Makes sure we dont overshoot our limits for left hang arm
    

    rightMotor.set(rightMotorPower);

    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.HANG) {
      rightMotor.stopMotor();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
