
package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Field;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.software.LimelightHelpers;
import frc.robot.util.software.LimelightHelpers.PoseEstimate;


public class AprilTags extends SubsystemBase {
  public static void injectVisionData(Map<String, Pose3d> cameraPoses, SwerveDrive swerveDrive) {
    List<LimelightHelpers.PoseEstimate> poseEstimates = cameraPoses.keySet().stream().map(LimelightHelpers::getBotPoseEstimate_wpiBlue).collect(Collectors.toList());
    
    HashMap<String, Object> bestPoseEstimate = new HashMap<>();
    bestPoseEstimate.put("pose", new Pose2d());
    bestPoseEstimate.put("timestamp", Timer.getFPGATimestamp());
    bestPoseEstimate.put("translationError", Double.MAX_VALUE);
    bestPoseEstimate.put("rotationError", Double.MAX_VALUE);
    bestPoseEstimate.put("tagCount", 0);

    //if (swerveDrive.getRotationalVelocity() > 2.0) return;

    int tagCount = 0;
    for (PoseEstimate poseEstimate : poseEstimates) {
      tagCount += poseEstimate.tagCount;
    }
    
    // if (tagCount <= 1) return;

    List<Pose2d> poses = new ArrayList<>();

    for (PoseEstimate poseEstimate : poseEstimates) {
      Pose2d pose2d = poseEstimate.pose.toPose2d();
      if (IntStream.of(LIMELIGHT.BLACKLISTED_APRILTAGS).anyMatch(x -> x == poseEstimate.primaryTagID)) continue;
      if (poseEstimate.tagCount == 0) continue;
      if (pose2d.getTranslation().getNorm() == 0.0) continue;
      if (pose2d.getRotation().getRadians() == 0.0) continue;
      if (Math.abs(poseEstimate.pose.getZ()) > 1) continue;
      if (Double.isNaN(poseEstimate.avgTagDist)) continue;
      // if (poseEstimate.avgTagDist > 6) continue;
      
      // if (poseEstimate.avgTagDist > 5) continue;
      if (pose2d.getX() < 0.0 || pose2d.getY() < 0.0 || pose2d.getX() > Field.LENGTH || pose2d.getY() > Field.WIDTH) continue;
      boolean canChangeHeading = false;
      if (swerveDrive.canZeroHeading() && (poseEstimate.tagCount >= 2 || RobotState.isDisabled())) {
        canChangeHeading = true;
      }

      canChangeHeading = canChangeHeading && swerveDrive.getPose().getTranslation().getDistance(pose2d.getTranslation()) < 1.0;
      if (canChangeHeading) LEDs.setState(LEDs.State.HAS_VISION_TARGET_SPEAKER);
      
      double rotationError = Units.degreesToRadians(15);
      if (!canChangeHeading) {
        rotationError = 9999999;
        pose2d = new Pose2d(
          pose2d.getTranslation(),
          swerveDrive.getPose(poseEstimate.timestampSeconds).getRotation()
        );
      }

      double translationError = Math.pow(Math.abs(poseEstimate.avgTagDist), 2.0) / Math.pow(poseEstimate.tagCount, 2) / 10;

      // if (RobotState.isAutonomous() && poseEstimate.tagCount <= 1) {
      //   continue;
      // }

      poses.add(pose2d);
      translationError += 0.5;
      
      if (translationError < (double) bestPoseEstimate.get("translationError") || rotationError < Units.degreesToRadians(360.0)) {
        bestPoseEstimate.put("pose", pose2d);
        bestPoseEstimate.put("timestamp", poseEstimate.timestampSeconds);
        bestPoseEstimate.put("translationError", translationError);
        bestPoseEstimate.put("rotationError", rotationError);
        bestPoseEstimate.put("tagCount", poseEstimate.tagCount);
      }
    }

    if ((int) bestPoseEstimate.get("tagCount") > 0) {
      // Logger.log("canChangeHeading", canChangeHeading);
      // Logger.log("rotationAccuracy", rotationError);
      // Logger.log("poseRotation", pose2d.getRotation().getDegrees());

      swerveDrive.addVisionMeasurement((Pose2d) bestPoseEstimate.get("pose"), (double) bestPoseEstimate.get("timestamp"), VecBuilder.fill((double) bestPoseEstimate.get("translationError"), (double) bestPoseEstimate.get("translationError"), (double) bestPoseEstimate.get("rotationError")));
    }

    SwerveDrive.getField().getObject("visionPosese").setPoses(poses);
    
  }

  public static void printConfig(Map<String, Pose3d> cameraPoses) {
    System.out.println(
"""


//////////////////////////////////////
///// LIMELIGHT POSITION CONFIG //////
//////////////////////////////////////
"""
    );
    for (Map.Entry<String, Pose3d> limelight : cameraPoses.entrySet()) {
      System.out.println(
        String.format(
          """
          ----- %s.local:5801 -----
            LL Forward: %.5f
            LL Right:   %.5f
            LL Up:      %.5f
            LL Roll:    %.5f
            LL Pitch:   %.5f
            LL Yaw:     %.5f
          """,
          limelight.getKey(),
          limelight.getValue().getTranslation().getX(),
          limelight.getValue().getTranslation().getY(),
          limelight.getValue().getTranslation().getZ(),
          Units.radiansToDegrees(limelight.getValue().getRotation().getX()),
          Units.radiansToDegrees(limelight.getValue().getRotation().getY()),
          Units.radiansToDegrees(limelight.getValue().getRotation().getZ())
        )
      );
    }
  }
}