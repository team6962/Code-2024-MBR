package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveDrive;

public class GoToPose extends Command {
  private Command command;
  private SwerveDrive swerveDrive;
  private Supplier<Pose2d> poseSupplier;

  public GoToPose(Supplier<Pose2d> poseSupplier, SwerveDrive swerveDrive) {
    this.poseSupplier = poseSupplier;
    this.swerveDrive = swerveDrive;
  }

  public void initialize() {
    Pose2d pose = poseSupplier.get();
    command = swerveDrive.goToSimple(pose);
    // command = swerveDrive.pathfindThenFollowPath(
    //   new Pose2d(new Translation2d(-1.0, 0).rotateBy(pose.getTranslation().minus(swerveDrive.getPose().getTranslation()).getAngle()).plus(pose.getTranslation()), pose.getRotation()),
    //   pose
    // );
    command.schedule();
  }

  public void execute() {

  }

  public void end() {
    command.cancel();
  }
}
