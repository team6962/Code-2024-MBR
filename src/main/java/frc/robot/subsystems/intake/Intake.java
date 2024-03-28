package frc.robot.subsystems.intake;

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



public class Intake extends SubsystemBase {
  private CANSparkMax motor;
  private State state;
 
  public static enum State {
    IN,
    SLOW_IN,
    SLOW_OUT,
    OFF
  }

  public Intake() {
    motor = new CANSparkMax(CAN.INTAKE, MotorType.kBrushless);

    SparkMaxUtil.configureAndLog(this, motor, false, CANSparkMax.IdleMode.kCoast);
    SparkMaxUtil.configureCANStatusFrames(motor, false, false);
    SparkMaxUtil.save(motor);
  }

  public Command setState(State state) {
    return runEnd(
      () -> this.state = state,
      () -> this.state = State.OFF
    );
  }
  
  
  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_INTAKE) return;
    if (RobotState.isDisabled()) {
      state = State.OFF;
    }
    switch(state) {
      case IN:
        motor.set(-Preferences.INTAKE.IN_POWER);
        break;
      case SLOW_OUT:
        motor.set(Preferences.INTAKE.SLOW_OUT_POWER);
        break;
      case SLOW_IN:
        motor.set(-Preferences.INTAKE.TO_SHOOTER_POWER);
        break;
      case OFF:
        motor.set(0);
        break;
    }
    
     
    if (RobotContainer.getVoltage() < VOLTAGE_LADDER.INTAKE) motor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
