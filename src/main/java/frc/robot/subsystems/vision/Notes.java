package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Field;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.drive.SwerveDrive;

public class Notes {
  public static Translation2d getNotePosition(String name, Rotation2d pitch, SwerveDrive swerveDrive, Translation2d fieldVelocity, Translation3d cameraToRobot) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    
    if (table.getEntry("tv").getDouble(0) == 0) return null;

    Translation2d notePosition = new Translation2d();
    double x = table.getEntry("tx").getDouble(0);
    double y = table.getEntry("ty").getDouble(0);// - Math.sqrt(Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() * Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() * target.ta)/2;


    y += table.getEntry("tvert").getDouble(0) / 2.0 / LIMELIGHT.NOTE_CAMERA_HEIGHT_PIXELS * LIMELIGHT.FOV_HEIGHT.getDegrees();
    
    if (Units.degreesToRadians(y) + pitch.getRadians() > 0) return null;
    
    double latency = (table.getEntry("tl").getDouble(0) + table.getEntry("cl").getDouble(0));
    double distance = ((cameraToRobot.getZ() - Units.inchesToMeters(0.521453)) / - Math.tan(Units.degreesToRadians(y - 3.59747) + pitch.getRadians())) - (Field.NOTE_LENGTH / 2.0) * 1.97022 + Units.inchesToMeters(4); // empirical offset

    Translation2d relativePosition = new Translation2d(
      distance * Math.cos(Units.degreesToRadians(x)),
      -distance * Math.sin(Units.degreesToRadians(x))
    ).plus(cameraToRobot.toTranslation2d());
    
    double timestamp = (table.getEntry("hb").getLastChange() / 1000000.0) - (latency / 1000.0);

    Pose2d robotPosition = swerveDrive.getPose(timestamp);
    notePosition = robotPosition.getTranslation().plus(relativePosition.rotateBy(robotPosition.getRotation()));
    
    if (!RobotState.isDisabled()) {
      LEDs.setState(LEDs.State.CAN_SEE_NOTE);
    }

    return notePosition;
  }
}
