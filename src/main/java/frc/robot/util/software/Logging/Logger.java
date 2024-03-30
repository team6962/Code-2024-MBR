package frc.robot.util.software.Logging;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.LOGGING;

public final class Logger {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("Logs");
  private static Map<String, Supplier<Object>> suppliers = new HashMap<String, Supplier<Object>>();
  private static Map<String, Object> values = new HashMap<String, Object>();
  private static ShuffleboardTab tab = Shuffleboard.getTab("Logging");
  private static GenericEntry loggingButton = tab.add("Enable Logging", true).withWidget(BuiltInWidgets.kToggleSwitch).withSize(1, 1).withPosition(0, 0).getEntry();
  
  private static Notifier notifier = new Notifier(
    () -> {
      try {
        Logger.logAll();
      } catch (Exception e) {
        
      }
    }
  );
  
  public static void startLog() {
    notifier.startPeriodic(LOGGING.LOGGING_PERIOD_MS / 1000.0);
  }

  public static boolean loggingEnabled() {
    return loggingButton.getBoolean(false);
  }

  private static void logAll() {
    if (!loggingButton.getBoolean(false)) return;

    for (String key : suppliers.keySet()) {
      Object supplied_value = suppliers.get(key).get();
      Object saved_value = values.get(key);
      if (supplied_value.equals(saved_value)) {
        continue;
      }
      try {
        log(key, supplied_value);
      } catch (IllegalArgumentException e) {
        System.out.println("[LOGGING] unknown type: " + supplied_value.getClass().getSimpleName());
      }
    }
    logRio("roboRio");
  }

  public static void autoLog(String key, Supplier<Object> supplier) {
    suppliers.put(key, supplier);
  }

  public static void autoLog(String key, Object obj) {
    autoLog(key, () -> obj);
  }

  public static void autoLog(SubsystemBase subsystem, String key, Supplier<Object> supplier) {
    autoLog(subsystem.getClass().getSimpleName() + "/" + key, supplier);
  }

  public static void log(String key, Object obj) {
    if (obj instanceof CANSparkMax) log(key, (CANSparkMax) obj);
    else if (obj instanceof RelativeEncoder) log(key, (RelativeEncoder) obj);
    else if (obj instanceof AHRS) log(key, (AHRS) obj);
    else if (obj instanceof Pose2d) log(key, (Pose2d) obj);
    else if (obj instanceof SwerveModuleState) log(key, (SwerveModuleState) obj);
    else if (obj instanceof SwerveModuleState[]) log(key, (SwerveModuleState[]) obj);
    else if (obj instanceof SwerveModulePosition[]) log(key, (SwerveModulePosition[]) obj);
    else if (obj instanceof CANStatus) log(key, (CANStatus) obj);
    else if (obj instanceof PowerDistribution) log(key, (PowerDistribution) obj);
    else if (obj instanceof Translation2d) log(key, (Translation2d) obj);
    else if (obj instanceof Translation3d) log(key, (Translation3d) obj);
    else {
      table.getEntry(key).setValue(obj);
      values.put(key, obj);
    };
  }

  public static void log(String path, Translation3d translation) {
    log(path, new double[] {translation.getX(), translation.getY(), translation.getZ()});
  }

  public static void log(String path, Translation2d translation) {
    log(path, new double[] {translation.getX(), translation.getY()});
  }

  public static void log(String path, CANcoder encoder) {
    log(path + "/absolutePosition", encoder.getAbsolutePosition());
    log(path + "/position", encoder.getPosition());
    log(path + "/velocity", encoder.getVelocity());
  }

  public static void log(String path, AHRS navX) {
    log(path + "/isAltitudeValid", navX.isAltitudeValid());
    log(path + "/isCalibrating", navX.isCalibrating());
    log(path + "/isConnected", navX.isConnected());
    log(path + "/isMagneticDisturbance", navX.isMagneticDisturbance());
    log(path + "/isMagnetometerCalibrated", navX.isMagnetometerCalibrated());
    log(path + "/isMoving", navX.isMoving());
    log(path + "/isRotating", navX.isRotating());
    log(path + "/actualUpdateRate", navX.getActualUpdateRate());
    log(path + "/firmwareVersion", navX.getFirmwareVersion());
    log(path + "/altitude", navX.getAltitude());
    log(path + "/angle", navX.getAngle());
    log(path + "/angleAdjustment", navX.getAngleAdjustment());
    log(path + "/compassHeading", navX.getCompassHeading());
    log(path + "/displacementX", navX.getDisplacementX());
    log(path + "/displacementY", navX.getDisplacementY());
    log(path + "/displacementZ", navX.getDisplacementZ());
    log(path + "/fusedHeading", navX.getFusedHeading());
    log(path + "/pitch", navX.getPitch());
    log(path + "/pressure", navX.getPressure());
    log(path + "/roll", navX.getRoll());
    log(path + "/yaw", navX.getYaw());
    log(path + "/temperature", navX.getTempC());
    log(path + "/velocityX", navX.getVelocityX());
    log(path + "/velocityY", navX.getVelocityY());
    log(path + "/velocityZ", navX.getVelocityZ());
    log(path + "/accelerationX", navX.getRawAccelX());
    log(path + "/accelerationY", navX.getRawAccelY());
    log(path + "/accelerationZ", navX.getRawAccelZ());
  }

  public static void log(String path, Pose2d pose) {
    log(path + "_radians", new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
    log(path + "_degrees", new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
  }

  public static void log(String path, SwerveModuleState swerveModuleState) {
    log(path + "/state", new double[] {
      swerveModuleState.angle.getRadians(), 
      swerveModuleState.speedMetersPerSecond
    });
  }

  public static void log(String path, SwerveModuleState[] swerveModuleStates) {
    log(path + "/states", new double[] {
      swerveModuleStates[0].angle.getRadians(), 
      swerveModuleStates[0].speedMetersPerSecond, 
      swerveModuleStates[1].angle.getRadians(), 
      swerveModuleStates[1].speedMetersPerSecond, 
      swerveModuleStates[2].angle.getRadians(), 
      swerveModuleStates[2].speedMetersPerSecond, 
      swerveModuleStates[3].angle.getRadians(), 
      swerveModuleStates[3].speedMetersPerSecond, 
    });
  }

  public static void log(String path, SwerveModulePosition[] swerveModulePositions) {
    log(path + "/positions", new double[] {
      swerveModulePositions[0].angle.getRadians(), 
      swerveModulePositions[0].distanceMeters, 
      swerveModulePositions[1].angle.getRadians(), 
      swerveModulePositions[1].distanceMeters, 
      swerveModulePositions[2].angle.getRadians(), 
      swerveModulePositions[2].distanceMeters, 
      swerveModulePositions[3].angle.getRadians(), 
      swerveModulePositions[3].distanceMeters, 
    });
  }

  public static void logRio(String path) {
    log(path + "/isBrownedOut", RobotController.isBrownedOut());
    log(path + "/isSysActive", RobotController.isSysActive());
    log(path + "/brownoutVoltage", RobotController.getBrownoutVoltage());
    log(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(path + "/batteryVoltage", RobotController.getBatteryVoltage());
    log(path + "/inputCurrent", RobotController.getInputCurrent());
    log(path + "/inputVoltage", RobotController.getInputVoltage());
    log(path + "/3V3Line/current", RobotController.getCurrent3V3());
    log(path + "/5VLine/current", RobotController.getCurrent5V());
    log(path + "/6VLine/current", RobotController.getCurrent6V());
    log(path + "/3V3Line/enabled", RobotController.getEnabled3V3());
    log(path + "/5VLine/enabled", RobotController.getEnabled5V());
    log(path + "/6VLine/enabled", RobotController.getEnabled6V());
    log(path + "/3V3Line/faultCount", RobotController.getFaultCount3V3());
    log(path + "/5VLine/faultCount", RobotController.getFaultCount5V());
    log(path + "/6VLine/faultCount", RobotController.getFaultCount6V());
    log(path + "/3V3Line/voltage", RobotController.getVoltage3V3());
    log(path + "/5VLine/voltage", RobotController.getVoltage5V());
    log(path + "/6VLine/voltage", RobotController.getVoltage6V());
    log(path + "/canStatus", RobotController.getCANStatus());
  }

  public static void log(String path, CANStatus canStatus) {
    log(path + "/busOffCount", canStatus.busOffCount);
    log(path + "/percentBusUtilization", canStatus.percentBusUtilization);
    log(path + "/receiveErrorCount", canStatus.receiveErrorCount);
    log(path + "/transmitErrorCount", canStatus.transmitErrorCount);
    log(path + "/txFullCount", canStatus.txFullCount);
  }

  public static void log(String path, PowerDistribution PDH) {
    log(path + "/faults", PDH.getFaults());
    log(path + "/canId", PDH.getModule());
    for (int i = 0; i <= 23; i++) {
      log(path + "/channels/channel" + i + "Current", PDH.getCurrent(i));
    }
    log(path + "/isSwitchableChannelOn", PDH.getSwitchableChannel());
    log(path + "/temperature", PDH.getTemperature());
    log(path + "/totalCurrent", PDH.getTotalCurrent());
    log(path + "/totalJoules", PDH.getTotalEnergy());
    log(path + "/totalWatts", PDH.getTotalPower());
    log(path + "/voltage", PDH.getVoltage());
  }

  public static void log(String path, PowerDistributionFaults faults) {
    log(path + "/brownout", faults.Brownout);
    log(path + "/canWarning", faults.CanWarning);

    for (int i = 0; i < 24; i++) {
      log(path + "/channel" + i + "BreakerFault", faults.getBreakerFault(i));
    }
    
    log(path + "/hardwareFault", faults.HardwareFault);
  }

  public static void log(String path, Object self, Class clazz) {    
    for (Class c: clazz.getDeclaredClasses()) {
      try {
        log(path + "/" + c.getSimpleName(), self, c);
      }
      catch (Exception e) {}
    }

    for (Field f: clazz.getDeclaredFields()) {
      try {
        log(path + "/" + f.getName(), f.get(self));
      }
      catch (Exception e) {}
    }
  }
}
