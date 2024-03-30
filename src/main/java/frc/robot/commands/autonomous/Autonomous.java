package frc.robot.commands.autonomous;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.Constants.Field;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.vision.Notes;
import frc.robot.util.software.Logging.Logger;

public class Autonomous extends Command {
  private RobotStateController controller;
  private SwerveDrive swerveDrive;
  private List<Integer> queuedNotes = List.of();
  private List<Integer> remainingNotes = List.of();
  
  private double noteAvoidRadius = (Field.NOTE_LENGTH / 2.0 + Constants.SWERVE_DRIVE.BUMPER_DIAGONAL / 2.0);
  private double notePickupDistance = Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0 - Field.NOTE_LENGTH / 2.0;
  private double noteAlignDistance = Constants.SWERVE_DRIVE.BUMPER_LENGTH / 2.0 + Field.NOTE_LENGTH;
  private double speakerShotDistance = 5.0;
  private double adjacentNoteBand = 1.0;
  private double minPathfindingDistance = 2.5;
  private Command runningCommand;

  private boolean simulatedNote = true;
  private boolean isFirstNote = true;
  public static boolean avoidPillars = true;

  public Translation2d targetedNote;
  public Translation2d visionNoise = new Translation2d();

  public Debouncer hasNoteDebouncer = new Debouncer(0.1, DebounceType.kFalling);
  
  // private static ShuffleboardTab tab = Shuffleboard.getTab("Autonomous Sim");
  // private static SimpleWidget hasNote = tab.add("has Note", true).withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(0, 0);
  
  private enum State {
    SHOOT,
    PICKUP
  }

  public State state;

  public Autonomous(RobotStateController controller, SwerveDrive swerveDrive, List<Integer> queuedNotes) {
    this.controller = controller;
    this.swerveDrive = swerveDrive;
    // numbers represent each note on the field, 0-7, with 0 representing close notes
    this.queuedNotes = queuedNotes;
    this.remainingNotes = new ArrayList<>(List.of(0, 1, 2, 3, 4, 5, 6, 7));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = null;
    simulatedNote = true;
    controller.setState(RobotStateController.State.LEAVE_AMP).withTimeout(1.5).schedule();
  }
  
 /**
   * 
   * @return 0-7, the closest note that is queued
   */
  public Integer getClosestNote() {
    List<Integer> closeNotes = queuedNotes.stream().filter(n -> n <= 2).collect(Collectors.toList());
    List<Integer> wingNotes = queuedNotes.stream().filter(n -> n > 2).collect(Collectors.toList());
    
    List<Integer> relevantNotes = closeNotes.isEmpty() ? wingNotes : closeNotes;
    Integer lowestNote = Collections.max(relevantNotes);
    Integer highestNote = Collections.min(relevantNotes);

    Translation2d closestPosition = swerveDrive.getPose().getTranslation().nearest(
      List.of(
        Field.NOTE_POSITIONS.get(lowestNote).get(),
        Field.NOTE_POSITIONS.get(highestNote).get()
      )
    );

    if (closestPosition.equals(Field.NOTE_POSITIONS.get(lowestNote).get())) {
      return lowestNote;
    } else {
      return highestNote;
    }
  }

  /** 
   * 
   * @return the nearest translation2d shot position 
   */
  public Translation2d getClosestShootingPosition() {
    List<Translation2d> shotPositions = Field.SHOT_POSITIONS.stream().map(Supplier::get).collect(Collectors.toList());
    return swerveDrive.getPose().getTranslation().nearest(shotPositions);
  }

  /**
   * Makes all remaining notes into obstacles with a wide radius to avoid. 
   * Addes all the obstacles to the Pathfinding class
   */
  public void addNoteObstacles() {
    // List<Pair<Translation2d, Translation2d>> dynamicObstacles = new ArrayList<>();
    // for (Integer note : remainingNotes) {
    //   if (note >= 3) continue;
    //   dynamicObstacles.add(
    //     Pair.of(
    //       Field.NOTE_POSITIONS.get(note).get().minus(new Translation2d(noteAvoidRadius, noteAvoidRadius)),
    //       Field.NOTE_POSITIONS.get(note).get().plus(new Translation2d(noteAvoidRadius, noteAvoidRadius))
    //     )
    //   );
    // }
    // Pathfinding.setDynamicObstacles(dynamicObstacles, swerveDrive.getPose().getTranslation());
  }

  /**
   * Clears all note obstacles in the pathfinding class 
   */
  public void clearNoteObstacles() {
    Pathfinding.setDynamicObstacles(new ArrayList<>(), swerveDrive.getPose().getTranslation());
  }

  /**
   * @param queuedNotePosition The note that you queued to pickup next
   * @return Translation2d of the note that is closest to the queuedNotePosition
   */
  public Translation2d getVisionNotePosition(Translation2d queuedNotePosition) {
    Translation2d measuredNotePosition = Notes.getNotePosition(LIMELIGHT.NOTE_CAMERA_NAME, LIMELIGHT.NOTE_CAMERA_PITCH, swerveDrive, swerveDrive.getFieldVelocity(), LIMELIGHT.NOTE_CAMERA_POSITION);

    Random random = new Random();
    
    if (RobotBase.isSimulation()) {
      visionNoise = visionNoise.plus(new Translation2d((random.nextDouble() - 0.5) / 100, (random.nextDouble() - 0.5) / 100));

      List<Translation2d> measuredNotePositions = new ArrayList<>();
      for (Integer note : List.of(0, 1, 2, 3, 4, 5, 6, 7)) {
        if (shouldSeeNote(note)) {
          measuredNotePositions.add(Field.NOTE_POSITIONS.get(note).get().plus(new Translation2d(random.nextDouble() / 100, random.nextDouble() / 100)));
        }
      }
      if (measuredNotePositions.isEmpty()) measuredNotePosition = null;
      else measuredNotePosition = swerveDrive.getPose().getTranslation().nearest(measuredNotePositions).plus(visionNoise);
    }

    FieldObject2d visibleNotes = SwerveDrive.getField().getObject("visibleNotes");
    List<Pose2d> poses = new ArrayList<>();
    visibleNotes.setPoses(poses);

    if (measuredNotePosition == null) return null;

    List<Translation2d> theoreticalNotePositions = Field.NOTE_POSITIONS.stream().map(Supplier::get).collect(Collectors.toList());
    Translation2d trueMeasuredNotePosition = measuredNotePosition;

    if ((measuredNotePosition.getX() > (Field.LENGTH / 2.0 + Field.NOTE_LENGTH * 2.0) && Constants.IS_BLUE_TEAM.get()) || (measuredNotePosition.getX() < (Field.LENGTH / 2.0 - Field.NOTE_LENGTH * 2.0) && !Constants.IS_BLUE_TEAM.get())) {
      // Translation2d relativeNotePosition = measuredNotePosition.minus(swerveDrive.getPose().getTranslation());
      // relativeNotePosition = relativeNotePosition.div(-relativeNotePosition.getX());
      // relativeNotePosition = relativeNotePosition.times(swerveDrive.getPose().getTranslation().getX() - Field.LENGTH / 2);
      // measuredNotePosition = relativeNotePosition.plus(swerveDrive.getPose().getTranslation());
      return null;
    }

    if ((measuredNotePosition.getX() > (Field.LENGTH / 2.0) && Constants.IS_BLUE_TEAM.get()) || (measuredNotePosition.getX() < (Field.LENGTH / 2.0) && !Constants.IS_BLUE_TEAM.get())) {
      Translation2d relativeNotePosition = measuredNotePosition.minus(swerveDrive.getPose().getTranslation());
      relativeNotePosition = relativeNotePosition.div(-relativeNotePosition.getX());
      relativeNotePosition = relativeNotePosition.times(swerveDrive.getPose().getTranslation().getX() - Field.LENGTH / 2);
      measuredNotePosition = relativeNotePosition.plus(swerveDrive.getPose().getTranslation());
    }
    
    Translation2d theoreticalNoteCounterpart = measuredNotePosition.nearest(theoreticalNotePositions);
    if (
      theoreticalNoteCounterpart.getDistance(queuedNotePosition) < 0.05 && 
      Math.abs(queuedNotePosition.getX() - trueMeasuredNotePosition.getX()) < 1.0 && 
      swerveDrive.getPose().getTranslation().getDistance(trueMeasuredNotePosition) < 4.0
    ) {
      poses.add(new Pose2d(measuredNotePosition, new Rotation2d()));
      visibleNotes.setPoses(poses);
      return measuredNotePosition;
    }
    return null;
  }

  public Command pickup(Translation2d notePosition) {
    if (hasNote()) return Commands.runOnce(() -> {});
    
    List<Translation2d> fieldNotePositions = Field.NOTE_POSITIONS.stream().map(Supplier::get).collect(Collectors.toList());
    // note Index represents 
    Integer noteIndex = fieldNotePositions.indexOf(notePosition.nearest(fieldNotePositions));

    avoidPillars = noteIndex != 2;

    boolean adjacent = Math.abs(swerveDrive.getPose().getX() - notePosition.getX()) < adjacentNoteBand && Math.abs(swerveDrive.getPose().getY() - notePosition.getY()) > adjacentNoteBand;
    
    Rotation2d heading = notePosition.minus(Field.SPEAKER.get().toTranslation2d()).getAngle();
    if (!inRange(notePosition) && !adjacent) {
      heading = notePosition.minus(swerveDrive.getPose().getTranslation()).getAngle();
    }

    Translation2d alignmentPoint = new Translation2d(-noteAlignDistance, 0).rotateBy(heading).plus(notePosition);
    Translation2d pickupPoint = new Translation2d(-notePickupDistance, 0).rotateBy(heading).plus(notePosition);
    
    // SwerveDrive.getField().getObject("bezier").setPoses(List.of(new Pose2d(alignmentPoint, heading), new Pose2d(pickupPoint, heading)));
    
    boolean pathfind = !adjacent && !tooClose(notePosition) && swerveDrive.getPose().getTranslation().getDistance(alignmentPoint) > 0.6;
    
    if (pathfind) {
      System.out.println("PATHFIND");
      return Commands.sequence(
        Commands.runOnce(() -> {
          if (!inRange(notePosition) && !adjacent) {
            swerveDrive.setRotationTargetOverrideFromPoint(() -> notePosition, Rotation2d.fromDegrees(0.0));
          } else {
            swerveDrive.setRotationTargetOverrideFromPoint(() -> Field.SPEAKER.get().toTranslation2d(), Rotation2d.fromDegrees(180.0));
          }
          addNoteObstacles();
        }),
        AutoBuilder.pathfindToPose(
          new Pose2d(alignmentPoint, heading),
          SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS
        )
        .until(() -> tooClose(notePosition))
      ).until(() -> hasNote()).finallyDo(
        () -> {
          clearNoteObstacles();
          swerveDrive.setRotationTargetOverrideFromPoint(null, new Rotation2d());
        }
      );
    }

    List<Translation2d> bezierPoints;

    SwerveDrive.getField().getObject("alignmentPoint").setPose(new Pose2d(alignmentPoint, new Rotation2d()));
    SwerveDrive.getField().getObject("pickupPoint").setPose(new Pose2d(pickupPoint, new Rotation2d()));


    if (
      (alignmentPoint.getDistance(Field.SPEAKER.get().toTranslation2d()) < swerveDrive.getPose().getTranslation().getDistance(Field.SPEAKER.get().toTranslation2d()) && 
      pickupPoint.getDistance(Field.SPEAKER.get().toTranslation2d()) > swerveDrive.getPose().getTranslation().getDistance(Field.SPEAKER.get().toTranslation2d()) &&
      (pickupPoint.getDistance(swerveDrive.getPose().getTranslation()) < noteAlignDistance - notePickupDistance ||
      alignmentPoint.getDistance(swerveDrive.getPose().getTranslation()) < noteAlignDistance - notePickupDistance))
      || alignmentPoint.getDistance(swerveDrive.getPose().getTranslation()) < Field.NOTE_LENGTH * 2.0
    ) {
      System.out.println("#1");
      bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(swerveDrive.getPose().getTranslation(), pickupPoint.minus(swerveDrive.getPose().getTranslation()).getAngle()),
        new Pose2d(pickupPoint, heading)
      );
    } else {
      System.out.println("#2");
      bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(swerveDrive.getPose().getTranslation(), alignmentPoint.minus(swerveDrive.getPose().getTranslation()).getAngle()),
        new Pose2d(alignmentPoint, heading),
        new Pose2d(pickupPoint, heading)
      );
      if (adjacent) {
        System.out.println("#3");
        System.out.println("ADJACENT");
        bezierPoints = PathPlannerPath.bezierFromPoses(
          new Pose2d(swerveDrive.getPose().getTranslation(), Field.SPEAKER.get().toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle()),
          new Pose2d(alignmentPoint, heading),
          new Pose2d(pickupPoint, heading)
        );
      }
    }

    System.out.println("STANDARD");

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      new GoalEndState(
        0.0,
        swerveDrive.getHeading(),
        true
      )
    );

    return Commands.sequence(
      Commands.runOnce(() -> {
        if (!inRange(notePosition) && !adjacent) {
          swerveDrive.setRotationTargetOverrideFromPoint(() -> notePosition, Rotation2d.fromDegrees(0.0));
        } else {
          swerveDrive.setRotationTargetOverrideFromPoint(() -> Field.SPEAKER.get().toTranslation2d(), Rotation2d.fromDegrees(180.0));
        }
        addNoteObstacles();
      }),
      AutoBuilder.followPath(path).andThen(
          Commands.runOnce(() -> simulatedNote = true),
          Commands.waitSeconds(0.25)
        ).raceWith(
          Commands.sequence(
            //Commands.waitUntil(() -> swerveDrive.getFuturePose().getTranslation().getDistance(notePosition) < 2.0),
            controller.setState(RobotStateController.State.INTAKE)
          ),
          Commands.waitUntil(() -> hasNote())
        ).andThen(
          Commands.runOnce(() -> {
            remainingNotes.remove(noteIndex);
            queuedNotes.remove(noteIndex);
          })
        )
    ).finallyDo(
      () -> {
        clearNoteObstacles();
        swerveDrive.setRotationTargetOverrideFromPoint(null, new Rotation2d());
      }
    );
  }

  public Command shoot() {
    if (!hasNote()) return Commands.runOnce(() -> {});

    Translation2d shootingPosition = getClosestShootingPosition();

    Logger.log("shootingPosition", shootingPosition);
    
    Command moveCommand = Commands.runOnce(() -> {});
    if (!nearSpeaker()) {
      moveCommand = Commands.runOnce(() -> clearNoteObstacles()).andThen(AutoBuilder.pathfindToPose(
        new Pose2d(shootingPosition, swerveDrive.getHeading()),
        SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS
      ).until(() -> !hasNote() || nearSpeaker()));
    } else if (!avoidPillars) {
      Rotation2d heading = swerveDrive.getPose().getTranslation().minus(Field.SPEAKER.get().toTranslation2d()).getAngle();
      Translation2d backupPoint = new Translation2d(-0.25, 0).rotateBy(heading).plus(swerveDrive.getPose().getTranslation());
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(swerveDrive.getPose().getTranslation(), backupPoint.minus(swerveDrive.getPose().getTranslation()).getAngle()),
        new Pose2d(backupPoint, heading)
      );

      PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
        new GoalEndState(
          0.0,
          swerveDrive.getHeading(),
          true
        )
      );

      moveCommand = Commands.runOnce(() -> clearNoteObstacles()).andThen(
        AutoBuilder.followPath(path)
      );
    }
    
    return Commands.sequence(
      Commands.runOnce(() -> {
        swerveDrive.setRotationTargetOverrideFromPoint(null, Rotation2d.fromDegrees(0.0));
      }),
      Commands.race(
        moveCommand
          .andThen(
            Commands.sequence(
              // Commands.waitSeconds(1.0).onlyIf(() -> isFirstNote),
              Commands.waitUntil(() -> inRange(swerveDrive.getPose().getTranslation())),
              Commands.run(() -> {
                if (controller.canShoot()) simulatedNote = false;
              }).until(() -> simulatedNote == false).onlyIf(() -> RobotBase.isSimulation()),
              controller.setState(RobotStateController.State.SHOOT).until(() -> !hasNote())
            ).onlyWhile(() -> nearSpeaker())
          ),
        controller.setState(RobotStateController.State.AIM_SPEAKER),
        controller.setState(RobotStateController.State.SPIN_UP)
      )
    ).until(() -> !hasNote()).finallyDo(() -> {
      isFirstNote = false;
      swerveDrive.setRotationTargetOverrideFromPoint(null, new Rotation2d());
    });
  }

  public boolean shouldSeeNote(int note) {
    Translation2d position = Field.NOTE_POSITIONS.get(note).get();
    Translation2d relativePosition = position.minus(swerveDrive.getPose().getTranslation()).rotateBy(swerveDrive.getPose().getRotation().unaryMinus());
    relativePosition = relativePosition.minus(Constants.LIMELIGHT.NOTE_CAMERA_POSITION.toTranslation2d());
    
    if (relativePosition.getX() < 0) return false;

    Rotation2d lateralAngleMin = Rotation2d.fromRadians(Math.atan((relativePosition.getY() - Field.NOTE_LENGTH / 2.0) / relativePosition.getX()));
    Rotation2d lateralAngleMax = Rotation2d.fromRadians(Math.atan((relativePosition.getY() + Field.NOTE_LENGTH / 2.0) / relativePosition.getX()));

    Rotation2d verticalAngle = Rotation2d.fromRadians(Math.atan((Constants.LIMELIGHT.NOTE_CAMERA_POSITION.getZ() - Field.NOTE_THICKNESS / 2.0) / relativePosition.getX()));
    verticalAngle = verticalAngle.plus(Constants.LIMELIGHT.NOTE_CAMERA_PITCH);

    if (verticalAngle.getDegrees() < -(Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() / 2.0) || verticalAngle.getDegrees() > (Constants.LIMELIGHT.FOV_HEIGHT.getDegrees() / 2.0)) return false;
    if (lateralAngleMin.getDegrees() < -(Constants.LIMELIGHT.FOV_WIDTH.getDegrees() / 2.0) || lateralAngleMax.getDegrees() > (Constants.LIMELIGHT.FOV_WIDTH.getDegrees() / 2.0)) return false;
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    List<Translation2d> fieldNotePositions = Field.NOTE_POSITIONS.stream().map(Supplier::get).collect(Collectors.toList());
        
    if (!RobotState.isAutonomous()) {
      runningCommand.cancel();
      this.cancel();
      return;
    }

    if ((state != State.SHOOT || !runningCommand.isScheduled()) && hasNote()) {
      if (runningCommand != null) runningCommand.cancel();
      runningCommand = shoot();
      runningCommand.schedule();
      state = State.SHOOT;
      System.out.println("SHOOTING");
      return;
    }

    if ((state != State.PICKUP || (state == State.PICKUP && !runningCommand.isScheduled())) && !hasNote() && !queuedNotes.isEmpty()) {
      state = State.PICKUP;
      if (runningCommand != null) runningCommand.cancel();
      Translation2d closestNote = Field.NOTE_POSITIONS.get(getClosestNote()).get();
      if (targetedNote == null || targetedNote.nearest(fieldNotePositions).getDistance(closestNote) > 0.01) {
        targetedNote = closestNote;
      }
      runningCommand = pickup(targetedNote);
      runningCommand.schedule();
      System.out.println("PICKUP");
      return;
    }

    if (state == State.PICKUP && !hasNote() && targetedNote != null) {
      Translation2d visionNotePosition = getVisionNotePosition(targetedNote.nearest(fieldNotePositions));
      boolean firstVisionPosition = targetedNote.nearest(fieldNotePositions).getDistance(targetedNote) < 0.01;
      if (visionNotePosition != null && (visionNotePosition.getDistance(targetedNote) > Field.NOTE_LENGTH / 4.0 * swerveDrive.getPose().getTranslation().getDistance(visionNotePosition) || firstVisionPosition)) {
        targetedNote = visionNotePosition;
        if (runningCommand != null) runningCommand.cancel();
        runningCommand = pickup(targetedNote);
        runningCommand.schedule();
      };
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (runningCommand != null) runningCommand.cancel();
    clearNoteObstacles();
    swerveDrive.setRotationTargetOverrideFromPoint(null, new Rotation2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean hasNote() {
    if (RobotBase.isSimulation()) {
      return hasNoteDebouncer.calculate(simulatedNote);
      // return hasNote.getEntry().getBoolean(false);
    } else {
      return hasNoteDebouncer.calculate(controller.hasNote());
    }
  }

  /**
   * Checks if the the position where robot picks up the note is too close to use 
   * pathfinding based on the future position of the robot and position of the note
   * 
   * @param notePosition Position of a note on the field
   * @return True, if the note is too close to the robot to use pathfinding
   */
  public boolean tooClose(Translation2d notePosition) {
    return swerveDrive.getFuturePose().getTranslation().getDistance(new Translation2d(-notePickupDistance, 0).rotateBy(notePosition.minus(Field.SPEAKER.get().toTranslation2d()).getAngle()).plus(notePosition)) < minPathfindingDistance;
  }

  public boolean nearSpeaker() {
    return !controller.underStage() && inRange(swerveDrive.getFuturePose().getTranslation());
  }

  public boolean inRange(Translation2d point) {
    return 
      point.getDistance(Field.SPEAKER.get().toTranslation2d()) < (point.getY() < Field.WIDTH / 2.0 ? 3.5 : speakerShotDistance) && 
      ((point.getX() < Field.WING_X.get() - 0.5 && Constants.IS_BLUE_TEAM.get()) || (point.getX() > Field.WING_X.get() + 0.5 && !Constants.IS_BLUE_TEAM.get()));
  }
}