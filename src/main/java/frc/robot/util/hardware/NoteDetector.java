package frc.robot.util.hardware;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants.NEO;
import frc.robot.Constants.Constants.NEO550;
import frc.robot.util.software.Logging.Logger;

public class NoteDetector extends SubsystemBase {
  int filterSize = 3;
  MedianFilter filter = new MedianFilter(filterSize);
  double delay = NEO.SAFE_RAMP_RATE * 1.0;
  double delayCounter = 0.0;
  CANSparkMax motor;
  double gearing = 0.0;
  double filteredTorque;
  double freeTorque;
  boolean isNeo550 = false;

  public NoteDetector(CANSparkMax motor, double gearing, double freeTorque, boolean isNeo550) {
    this.motor = motor;
    this.gearing = gearing;
    this.freeTorque = freeTorque;
    this.isNeo550 = isNeo550;
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/isNoteStatusTrue", () -> isNoteStatus(true));
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/isNoteStatusFalse", () -> isNoteStatus(false));
    Logger.autoLog("NoteDetectors/" + motor.getDeviceId() + "/appliedTorque", () -> filteredTorque);
  }

  @Override
  public void periodic() {
    filteredTorque = 0.0;

    double output = motor.getAppliedOutput();
    
    if (output == 0.0) {
      delayCounter = 0.0;
      filter.reset();
      return;
    }

    if (delayCounter <= delay) {
      delayCounter += Robot.getLoopTime();
      filter.reset();
      return;
    }

    double motorTorque = NEO.STATS.stallTorqueNewtonMeters / NEO.STATS.stallCurrentAmps * motor.getOutputCurrent();
    if (isNeo550) {
      motorTorque = NEO550.STATS.stallTorqueNewtonMeters / NEO550.STATS.stallCurrentAmps * motor.getOutputCurrent();
    }
    double appliedTorque = motorTorque * gearing;
    filteredTorque = filter.calculate(appliedTorque);
  }

  public boolean isNoteStatus(boolean status) {
    if (filteredTorque == 0.0) return false;
    return ((filteredTorque - freeTorque) > 0.0) == status;
  }
}