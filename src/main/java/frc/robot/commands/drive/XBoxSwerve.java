// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.Preferences;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.util.software.MathUtils;
import frc.robot.util.software.MathUtils.InputMath;

public class XBoxSwerve extends Command {
  private XboxController controller;
  private SwerveDrive swerveDrive;
  private RobotStateController stateController;

  public final double MAX_DRIVE_VELOCITY = SwerveModule.calcWheelVelocity(Preferences.SWERVE_DRIVE.TELEOPERATED_BOOST_POWER);
  public final double NOMINAL_DRIVE_VELOCITY = SwerveModule.calcWheelVelocity(Preferences.SWERVE_DRIVE.TELEOPERATED_DRIVE_POWER);
  public final double FINE_TUNE_DRIVE_VELOCITY = SwerveModule.calcWheelVelocity(Preferences.SWERVE_DRIVE.TELEOPERATED_FINE_TUNE_DRIVE_POWER);
  public final double NOMINAL_ANGULAR_VELOCITY = SwerveDrive.toAngular(SwerveModule.calcWheelVelocity(Preferences.SWERVE_DRIVE.TELEOPERATED_ROTATE_POWER));
  public final double MAX_ANGULAR_VELOCITY = SwerveDrive.toAngular(MAX_DRIVE_VELOCITY); // TODO: use physics from constants file
  
  private Translation2d velocity = new Translation2d();
  private double angularVelocity = 0.0;
  
  public XBoxSwerve(SwerveDrive swerveDrive, XboxController xboxController, RobotStateController stateController) {
    this.swerveDrive = swerveDrive;
    this.controller = xboxController;
    this.stateController = stateController;
    // controller.setRumble(RumbleType.kBothRumble, 1.0);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotState.isTeleop()) return;

    // Disable drive if the controller disconnects
    if (!controller.isConnected()) {
      swerveDrive.stopModules();
      return;
    }

    double leftTrigger = controller.getLeftTriggerAxis();
    double rightTrigger = controller.getRightTriggerAxis();
    Translation2d leftStick = new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    Translation2d rightStick = new Translation2d(controller.getRightX(), -controller.getRightY());
    
    if (RobotBase.isSimulation()) {
      leftStick = new Translation2d(controller.getRawAxis(0), -controller.getRawAxis(1));
      rightStick = new Translation2d(controller.getRawAxis(2), -controller.getRawAxis(3));
      leftTrigger = controller.getRawAxis(5);
      rightTrigger = controller.getRawAxis(4);
    }

    // Deadbands
    leftStick = InputMath.addCircularDeadband(leftStick, 0.1);
    rightStick = InputMath.addCircularDeadband(rightStick, 0.1);


    angularVelocity += -rightStick.getX() * MathUtils.map(rightTrigger, 0, 1, NOMINAL_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    
    velocity = velocity.plus(leftStick.times(MathUtils.map(rightTrigger, 0, 1, NOMINAL_DRIVE_VELOCITY, MAX_DRIVE_VELOCITY)));
    
    if (leftTrigger > 0.5 && velocity.getNorm() > 0) {
      velocity = velocity.div(velocity.getNorm()).times(Preferences.SWERVE_DRIVE.TELEOPERATED_SHOOTER_SPEED);
    }

    if (controller.getPOV() != -1) {
      Translation2d povVelocity = new Translation2d(Math.cos(Units.degreesToRadians(controller.getPOV())) * FINE_TUNE_DRIVE_VELOCITY, -Math.sin(Units.degreesToRadians(controller.getPOV())) * FINE_TUNE_DRIVE_VELOCITY);
      velocity = velocity.plus(povVelocity);
    }

    // Zero heading when Y is pressed
    if (controller.getYButton()) {
      Rotation2d newHeading = new Rotation2d();
      if (!Constants.IS_BLUE_TEAM.get()) {
        newHeading = Rotation2d.fromDegrees(180.0);
      }
      swerveDrive.resetGyroHeading(newHeading); 
    }

    if (controller.getAButton()) {
      // swerveDrive.goToNearestPose(List.of(Field.AUTO_MOVE_POSITIONS.values().toArray(new Pose2d[] {})), controller).schedule();
    }

    if (RobotBase.isSimulation()) {
      if (Constants.IS_BLUE_TEAM.get()) {
        velocity = velocity.rotateBy(Rotation2d.fromDegrees(90.0));
      } else {
        velocity = velocity.rotateBy(Rotation2d.fromDegrees(-90.0));
      }
    }

    swerveDrive.driveFieldRelative(velocity.getX(), velocity.getY(), angularVelocity);
    // if (leftStick.getNorm() > 0.05 && (controller.getLeftBumper() || controller.getRightBumper())) {
    //   swerveDrive.setTargetHeading(leftStick.getAngle());
    // }
    
    angularVelocity = 0.0;
    velocity = new Translation2d();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static Translation2d avoidObstacles(Translation2d velocity, SwerveDrive swerveDrive) {
    if (!Constants.IS_BLUE_TEAM.get()) {
      velocity = velocity.rotateBy(Rotation2d.fromDegrees(180.0));
    }

    double mag = velocity.getNorm();
    List<Translation2d> stagePillars = new ArrayList<>();
    for (Translation2d pillar : Field.BLUE_STAGE_CORNERS) stagePillars.add(pillar);
    for (Translation2d pillar : Field.RED_STAGE_CORNERS) stagePillars.add(pillar);
    for (Translation2d pillar : stagePillars) {
      double bubbleRadius = 
        Math.hypot(Units.inchesToMeters(6.0), Units.inchesToMeters(6.0)) + 
        Constants.SWERVE_DRIVE.BUMPER_DIAGONAL / 2.0 +
        (swerveDrive.getPose().getTranslation().getDistance(pillar) - swerveDrive.getFuturePose().getTranslation().getDistance(pillar)) * 2.0;
      if (bubbleRadius <= 0) continue;
      if (swerveDrive.getPose().getTranslation().getDistance(pillar) > bubbleRadius) continue;
      Translation2d force = swerveDrive.getPose().getTranslation().minus(pillar);
      force = force.div(force.getNorm()).times(mag);
      velocity = velocity.plus(force);
    }
    if (mag != 0.0) {
      velocity = velocity.div(velocity.getNorm()).times(mag);
    }

    if (!Constants.IS_BLUE_TEAM.get()) {
      velocity = velocity.rotateBy(Rotation2d.fromDegrees(-180.0));
    }

    // if (
    //   (swerveDrive.getFuturePose().getX() > Field.LENGTH - Constants.SWERVE_DRIVE.BUMPER_DIAGONAL / 2.0 && swerveDrive.getFieldVelocity().getX() > 0.05) || 
    //   (swerveDrive.getFuturePose().getY() > Field.WIDTH - Constants.SWERVE_DRIVE.BUMPER_DIAGONAL / 2.0 && swerveDrive.getFieldVelocity().getY() > 0.05) || 
    //   (swerveDrive.getFuturePose().getX() < Constants.SWERVE_DRIVE.BUMPER_DIAGONAL / 2.0 && swerveDrive.getFieldVelocity().getX() < -0.05) || 
    //   (swerveDrive.getFuturePose().getY() < Constants.SWERVE_DRIVE.BUMPER_DIAGONAL / 2.0 && swerveDrive.getFieldVelocity().getY() < -0.05) ||
    //   (MathUtils.isInsideTriangle(Field.RED_SOURCE_AVOID_CORNERS[0], Field.RED_SOURCE_AVOID_CORNERS[1], Field.RED_SOURCE_AVOID_CORNERS[2], swerveDrive.getFuturePose().getTranslation()) && (swerveDrive.getFieldVelocity().getX() < -0.05 || swerveDrive.getFieldVelocity().getY() < -0.05)) ||
    //   (MathUtils.isInsideTriangle(Field.BLUE_SOURCE_AVOID_CORNERS[0], Field.BLUE_SOURCE_AVOID_CORNERS[1], Field.BLUE_SOURCE_AVOID_CORNERS[2], swerveDrive.getFuturePose().getTranslation()) && (swerveDrive.getFieldVelocity().getX() > 0.05 || swerveDrive.getFieldVelocity().getY() < -0.05))
    // ) {
    //   velocity = velocity.div(5.0);
    //   // velocity = new Translation2d();
    // }
    return velocity;
  }
}