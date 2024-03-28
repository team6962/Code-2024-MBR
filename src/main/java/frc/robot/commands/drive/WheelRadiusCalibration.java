package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.drive.SwerveDrive;

public class WheelRadiusCalibration extends Command {
  private SwerveDrive swerveDrive;
  private Rotation2d lastGyroHeading;
  private SwerveModulePosition[] startingPositions;
  private double gyroRadians;
  private double timer;

  public WheelRadiusCalibration(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  public void initialize() {
    startingPositions = swerveDrive.getModulePositions();
    lastGyroHeading = swerveDrive.getHeading();
    gyroRadians = 0.0;
    timer = 0.0;
  }

  public void execute() {
    timer += Robot.getLoopTime();
    if (timer > 10.0) {
      swerveDrive.driveRobotRelative(0, 0, 0.0);
    } else {
      swerveDrive.driveRobotRelative(0, 0, SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY / 3.0);
    }
    
    gyroRadians += Math.abs(swerveDrive.getHeading().minus(lastGyroHeading).getRadians());
    lastGyroHeading = swerveDrive.getHeading();

    SwerveModulePosition[] modulePositions = swerveDrive.getModulePositions();
    double averageRadiansDriven = 0;
    for (int i = 0; i < startingPositions.length; i++) {
      SwerveModulePosition startingPosition = startingPositions[i];
      SwerveModulePosition modulePosition = modulePositions[i];
      averageRadiansDriven += Math.abs(modulePosition.distanceMeters - startingPosition.distanceMeters) / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
    }
    averageRadiansDriven /= startingPositions.length;

    System.out.println("WHEEL RADIUS (IN): " + Units.metersToInches(SWERVE_DRIVE.WHEEL_RADIUS * (gyroRadians / averageRadiansDriven)));
  }
}
