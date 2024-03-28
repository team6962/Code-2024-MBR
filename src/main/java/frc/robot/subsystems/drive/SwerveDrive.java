// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.LIMELIGHT;
import frc.robot.Constants.Constants.SWERVE_DRIVE;
import frc.robot.Constants.Field;
import frc.robot.commands.autonomous.Autonomous;
import frc.robot.commands.drive.XBoxSwerve;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.vision.AprilTags;
import frc.robot.subsystems.vision.Notes;
import frc.robot.util.software.CustomSwerveDrivePoseEstimator;
import frc.robot.util.software.MathUtils;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

/**
 * This class represents the subsystem for the swerve drive. It contains four
 * swerve module objects and a gyroscope object.
 */
public class SwerveDrive extends SubsystemBase {
  public SwerveModule[] modules = new SwerveModule[SWERVE_DRIVE.MODULE_COUNT];
  private static AHRS gyro;

  private SwerveDriveKinematics kinematics = getKinematics();
  private CustomSwerveDrivePoseEstimator poseEstimator;
  private static Field2d field = new Field2d();
  private Rotation2d gyroHeading = Rotation2d.fromDegrees(0.0);
  private Rotation2d gyroOffset = SWERVE_DRIVE.STARTING_POSE.get().getRotation();
  private Debouncer doneRotating = new Debouncer(0.5);

  private ChassisSpeeds drivenChassisSpeeds = new ChassisSpeeds();

  private PIDController alignmentController = new PIDController(
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kP,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kI,
    SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.kD
  );
  private double addedAlignmentAngularVelocity = 0.0;

  private boolean isAligning = false;
  private boolean parked = false;
  private boolean parkingDisabled = false;
  private boolean isDriven = false;
  private boolean gyroConnected = false;

  private Supplier<Translation2d> rotationOverridePoint = null;
  private Rotation2d rotationOverrideOffset = new Rotation2d();

  private SWERVE_DRIVE.MODULE_CONFIG[] equippedModules;

  private SwerveDriveWheelPositions previousWheelPositions;

  public SwerveDrive() {
    // Create the serve module objects
    if (SWERVE_DRIVE.IS_PROTOTYPE_CHASSIS) {
      equippedModules = SWERVE_DRIVE.EQUIPPED_MODULES_PROTOTYPE;
    } else {
      equippedModules = SWERVE_DRIVE.EQUIPPED_MODULES_COMPETITION;
    }

    int corner = 0;
    for (SWERVE_DRIVE.MODULE_CONFIG config : equippedModules) {
      String name = SWERVE_DRIVE.MODULE_NAMES[corner];
      if (RobotBase.isSimulation()) {
        modules[corner] = new SwerveModuleSim(config, corner, name);
      } else {
        modules[corner] = new SwerveModule(config, corner, name);
      }
      corner++;
    }

    // Set up pose estimator and rotation controller
    Logger.log("swerveModulePositions", getModulePositions());
    poseEstimator = new CustomSwerveDrivePoseEstimator(
      kinematics,
      SWERVE_DRIVE.STARTING_POSE.get().getRotation(),
      getModulePositions(),
      SWERVE_DRIVE.STARTING_POSE.get(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2)),
      VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(30))
    );

    previousWheelPositions = new SwerveDriveWheelPositions(getModulePositions());

    alignmentController.enableContinuousInput(-Math.PI, Math.PI);
    alignmentController.setTolerance(SWERVE_DRIVE.ABSOLUTE_ROTATION_GAINS.TOLERANCE.getRadians());
    // setTargetHeading(SWERVE_DRIVE.STARTING_POSE.getRotation());
    
    // If possible, connect to the gyroscope
    try {
      gyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), false);
    }

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyroOffset = gyroOffset.minus(gyro.getRotation2d());
      } catch (Exception e) {}
    }).start();
    
    SmartDashboard.putData("Field", field);
    
    Logger.autoLog("SwerveDrive/pose", () -> this.getPose());
    Logger.autoLog("SwerveDrive/measuredHeading", () -> this.getHeading().getDegrees());
    Logger.autoLog("SwerveDrive/targetHeading", () -> Units.radiansToDegrees(alignmentController.getSetpoint()));
    Logger.autoLog("SwerveDrive/targetStates", this::getTargetModuleStates);
    Logger.autoLog("SwerveDrive/measuredStates", this::getMeasuredModuleStates);
    
    StatusChecks.addCheck(this, "isGyroConnected", gyro::isConnected);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getMeasuredChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD), // Translation PID constants
        new PIDConstants(SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kP, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kI, SWERVE_DRIVE.AUTONOMOUS.   ROTATION_GAINS.kD), // Rotation PID constants
        SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY, // Max module speed, in m/s
        SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
      ),
      this::shouldFlipPaths,
      this // Reference to this subsystem to set requirements
    );
    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        field.getObject("Target Pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        // Do whatever you want with the poses here
        field.getObject("Active Path").setPoses(poses);
    });
  }

  // List<Pose2d> shotPoses = new ArrayList<>();

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_DRIVE) return;
    if (RobotState.isDisabled()) {
      for (SwerveModule module : modules) {
        module.seedSteerEncoder();
      }
      setTargetHeading(getHeading());
      isAligning = false;
      rotationOverridePoint = null;
    }
    
    Logger.log("Loop-Time", Robot.getLoopTime());
    
    updateOdometry();


    // System.out.println(Constants.SWERVE_DRIVE.PHYSICS.SLIPLESS_CURRENT_LIMIT);
    // System.out.println(Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION);
    // System.out.println(Constants.SWERVE_DRIVE.ROBOT_MASS);
    // System.out.println(Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY);


    // List<Translation2d> notePositions = Notes.getNotePositions(LIMELIGHT.NOTE_CAMERA_NAME, LIMELIGHT.NOTE_CAMERA_PITCH, this, getFieldVelocity(), LIMELIGHT.NOTE_CAMERA_POSITION);
    // SwerveDrive.getField().getObject("notes").setPoses(notePositions.stream().map(p -> new Pose2d(p, new Rotation2d())).toList());

    
    // List<Translation2d> notePositions = Notes.getNotePositions(LIMELIGHT.NOTE_CAMERA_NAME, LIMELIGHT.NOTE_CAMERA_PITCH, this, getFieldVelocity(), LIMELIGHT.NOTE_CAMERA_POSITION);
    // SwerveDrive.getField().getObject("notes").setPoses(notePositions.stream().map(p -> new Pose2d(p, new Rotation2d())).toList());
    // SwerveDrive.getField().getObject("futurePosition").setPose(getFuturePose());

    // System.out.println(Constants.SHOOTER_WHEELS.PROFILE.kV);

    FieldObject2d visibleNotes = SwerveDrive.getField().getObject("visibleNotes");
    
    // List<Pose2d> poses = new ArrayList<>();
    // for (Integer note1 : List.of(0, 1, 2, 3, 4, 5, 6, 7)) {
    //   Translation2d position = Field.NOTE_POSITIONS[note1];
    //   Translation2d relativePosition = position.minus(getPose().getTranslation()).rotateBy(getPose().getRotation().unaryMinus());
    //   poses.add(new Pose2d(relativePosition, new Rotation2d()));
    // }

    Translation2d notePosition = Notes.getNotePosition(LIMELIGHT.NOTE_CAMERA_NAME, LIMELIGHT.NOTE_CAMERA_PITCH, this, getFieldVelocity(), LIMELIGHT.NOTE_CAMERA_POSITION);
    if (notePosition != null) {
      visibleNotes.setPose(new Pose2d(notePosition, new Rotation2d()));
    }

    // Pose2d randomPose = new Pose2d(
    //   new Translation2d(
    //     Field.LENGTH * Math.random(),
    //     Field.WIDTH * Math.random()
    //   ),
    //   new Rotation2d()
    // );

    // randomPose = new Pose2d(randomPose.getTranslation(), randomPose.getTranslation().minus(Field.SPEAKER.toTranslation2d()).getAngle());

    // boolean inRange = ShooterMath.inRange(Field.SPEAKER, randomPose, Preferences.SHOOTER_WHEELS.TARGET_SPEED);
    // if (inRange) {
    //   shotPoses.add(randomPose);
    // }
    // FieldObject2d shotSpots = SwerveDrive.getField().getObject("shotSpots");
    // shotSpots.setPoses(shotPoses);

    // System.out.println(ShooterMath.calcPivotAngle(Field.SPEAKER, getPose(), Preferences.SHOOTER_WHEELS.TARGET_SPEED));

    // Update current heading based on gyroscope or wheel speeds

    // Update pose based on measured heading and swerve module positions
    
    // Update field
    FieldObject2d modulesObject = field.getObject("Swerve Modules");

    // Update swerve module poses
    Pose2d[] modulePoses = new Pose2d[SWERVE_DRIVE.MODULE_COUNT];
    Pose2d robotPose = getPose();
    
    int i = 0;
    for (SwerveModule module : modules) {
      modulePoses[i] = module.getPose(robotPose);
      i++;
    }

    modulesObject.setPoses(modulePoses);

    // Update robot pose
    field.setRobotPose(getPose());

    if (!isDriven) {
      driveFieldRelative(0.0, 0.0, 0.0);
    }
    isDriven = false;
  }

  public void updateOdometry() {
    Logger.log("gyro.isConnected()", gyro.isConnected());
    Logger.log("gyro.getLastSensorTimestamp()", gyro.getLastSensorTimestamp());
    Logger.log("gyro.isCalibrating()", gyro.isCalibrating());
    Logger.log("gyro.getRotation2d()", gyro.getRotation2d().getDegrees());

    Pose2d poseBefore = getPose();

    SwerveDriveWheelPositions wheelPositions = new SwerveDriveWheelPositions(getModulePositions());
    Twist2d twist = kinematics.toTwist2d(previousWheelPositions, wheelPositions);
    Pose2d newPose = getPose().exp(twist);


    if (!gyroConnected && (gyro.isConnected() && !gyro.isCalibrating())) {
      gyroOffset = gyroHeading.minus(gyro.getRotation2d());
    }
    gyroConnected = gyro.isConnected() && !gyro.isCalibrating();

    if (gyroConnected && !RobotBase.isSimulation()) {
      gyroHeading = gyro.getRotation2d();
    } else {
      gyroHeading = gyroHeading.plus(newPose.getRotation().minus(getPose().getRotation()));
    }

    Logger.log("swerveModulePositions", getModulePositions());
    poseEstimator.update(gyroHeading.plus(gyroOffset), getModulePositions());
    AprilTags.injectVisionData(LIMELIGHT.APRILTAG_CAMERA_POSES, this);

    Pose2d currentPose = getPose();
    double magnitude = currentPose.getTranslation().getNorm();
    if (magnitude > 1000 || Double.isNaN(magnitude) || Double.isInfinite(magnitude)) {
      System.out.println("BAD");
      LEDs.setState(LEDs.State.BAD);
      resetPose(gyroHeading.plus(gyroOffset), poseBefore, previousWheelPositions);
    }
    
    previousWheelPositions = wheelPositions.copy();
  }

  @Override
  public void simulationPeriodic() {
    // Simulate the correct voltage going into the roboRIO
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(getCurrent()));
  }

  /**
   * Drives the robot at a given field-relative velocity
   * @param xVelocity [meters / second] Positive x is away from your alliance wall
   * @param yVelocity [meters / second] Positive y is to your left when standing behind the alliance wall
   * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
   */
  public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveFieldRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  /**
   * 
   * Drives the robot at a given field-relative ChassisSpeeds
   * @param fieldRelativeSpeeds
   */
  private void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveAttainableSpeeds(fieldRelativeSpeeds);
  }

  /**
   * Drives the robot at a given robot-relative velocity
   * @param xVelocity [meters / second] Positive x is towards the robot's front
   * @param yVelocity [meters / second] Positive y is towards the robot's left
   * @param angularVelocity [radians / second] Rotational velocity, positive spins counterclockwise
   */
  public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
    driveRobotRelative(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity));
  }

  /**
   * Drives the robot at a given robot-relative ChassisSpeeds
   * @param robotRelativeSpeeds
   */
  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getAllianceAwareHeading()));
  }

  private void driveAttainableSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
    isDriven = true;

    if (!(RobotState.isAutonomous() && !Autonomous.avoidPillars)) {
      Translation2d velocity = XBoxSwerve.avoidObstacles(new Translation2d(
        fieldRelativeSpeeds.vxMetersPerSecond,
        fieldRelativeSpeeds.vyMetersPerSecond
      ), this);
      fieldRelativeSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), fieldRelativeSpeeds.omegaRadiansPerSecond);
    }

    if (fieldRelativeSpeeds.omegaRadiansPerSecond > 0 && !RobotState.isAutonomous()) {
      rotationOverridePoint = null;
    }

    if (rotationOverridePoint != null || RobotState.isAutonomous()) {
      fieldRelativeSpeeds.omegaRadiansPerSecond = 0.0;
      if (rotationOverridePoint != null) facePoint(rotationOverridePoint.get(), rotationOverrideOffset);
    }
    
    if (Math.abs(fieldRelativeSpeeds.omegaRadiansPerSecond) > 0.01) {
      setTargetHeading(getHeading());
      isAligning = false;
    }
    if (!isAligning && doneRotating.calculate(Math.abs(getDrivenChassisSpeeds().omegaRadiansPerSecond) < 0.1)) {
      setTargetHeading(getHeading());
      isAligning = true;
    }
    
    Logger.log("addedAlignmentAngularVelocity", addedAlignmentAngularVelocity);
    Logger.log("alignmentController.getSetpoint()", alignmentController.getSetpoint());

    double alignmentAngularVelocity = alignmentController.calculate(getHeading().getRadians()) + addedAlignmentAngularVelocity;
    addedAlignmentAngularVelocity = 0.0;
    if (isAligning && !alignmentController.atSetpoint() && !parked) fieldRelativeSpeeds.omegaRadiansPerSecond += alignmentAngularVelocity;
    
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getAllianceAwareHeading()));
    SwerveDriveKinematics.desaturateWheelSpeeds(
      moduleStates,
      fieldRelativeSpeeds,
      SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
      SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
      SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY
    );
    fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(moduleStates), getAllianceAwareHeading());

    // Limit translational acceleration
    Translation2d targetLinearVelocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    Translation2d currentLinearVelocity = new Translation2d(drivenChassisSpeeds.vxMetersPerSecond, drivenChassisSpeeds.vyMetersPerSecond);
    Translation2d linearAcceleration = (targetLinearVelocity).minus(currentLinearVelocity).div(Robot.getLoopTime());
    double linearForce = linearAcceleration.getNorm() * SWERVE_DRIVE.ROBOT_MASS;

     // Limit rotational acceleration
    double targetAngularVelocity = fieldRelativeSpeeds.omegaRadiansPerSecond;
    double currentAngularVelocity = drivenChassisSpeeds.omegaRadiansPerSecond;
    double angularAcceleration = (targetAngularVelocity - currentAngularVelocity) / Robot.getLoopTime();
    double angularForce = Math.abs((SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA * angularAcceleration) / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS);
    
    double frictionForce = 9.80 * SWERVE_DRIVE.ROBOT_MASS * SWERVE_DRIVE.FRICTION_COEFFICIENT;

    if (linearForce + angularForce > frictionForce) {
      double factor = (linearForce + angularForce) / frictionForce;
      linearAcceleration = linearAcceleration.div(factor);
      angularAcceleration /= factor;
    }

    Translation2d attainableLinearVelocity = currentLinearVelocity.plus(linearAcceleration.times(Robot.getLoopTime()));
    double attainableAngularVelocity = currentAngularVelocity + (angularAcceleration * Robot.getLoopTime());

    drivenChassisSpeeds = new ChassisSpeeds(attainableLinearVelocity.getX(), attainableLinearVelocity.getY(), attainableAngularVelocity);
    
    drivenChassisSpeeds = ChassisSpeeds.discretize(drivenChassisSpeeds, Robot.getLoopTime());

    SwerveModuleState[] drivenModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(drivenChassisSpeeds, getAllianceAwareHeading()));
    
    boolean moving = false;
    for (SwerveModuleState moduleState : kinematics.toSwerveModuleStates(fieldRelativeSpeeds)) if (Math.abs(moduleState.speedMetersPerSecond) > 0.0) moving = true;
    for (SwerveModuleState moduleState : drivenModuleStates) if (Math.abs(moduleState.speedMetersPerSecond) > 0.0) moving = true;
    // if (!parked) {
    //   for (SwerveModuleState moduleState : getMeasuredModuleStates()) if (Math.abs(moduleState.speedMetersPerSecond) > 0.05) moving = true;
    // }
    parked = false;

    if (!moving) {
      if (!parkingDisabled) {
        parkModules();
        return;
      }
      if (alignmentController.atSetpoint()) {
        parkForAlignment();
      }
    }

    parkingDisabled = false;
    
    driveModules(drivenModuleStates);
  }
  
  /**
   * Drives the swerve modules at the calculated speeds
   * @param moduleStates The calculated speeds and directions for each module
   */
  private void driveModules(SwerveModuleState[] moduleStates) {
    // Drive the swerve modules at the calculated speeds
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      modules[i].setTargetState(moduleStates[i]);
    }
  }

  /**
   * Sets the target heading for the robot
   * @param heading The target heading for the robot
   */
  public void setTargetHeading(Rotation2d heading) {
    alignmentController.setSetpoint(heading.getRadians());
    parkingDisabled = true;
  }

  /**
   * Sets the target heading for the robot
   * @param heading The target heading for the robot
   */
  public void setTargetHeadingAndVelocity(Rotation2d heading, double velocity) {
    setTargetHeading(heading);
    addedAlignmentAngularVelocity = velocity;
  }

  /**
   * Faces a point on the field
   * @param point The point on the field we want to face
   */
  public Command facePointCommand(Supplier<Translation2d> point, Rotation2d rotationOffset) {
    return Commands.run(
      () -> facePoint(point.get(), rotationOffset)
    );
  }

  public void facePoint(Translation2d point, Rotation2d rotationOffset) {
    double time = 0.02;

    if (point == null) {
      setTargetHeadingAndVelocity(getHeading(), 0.0);
      return;
    }


    Translation2d currentPosition = getPose().getTranslation();
    Translation2d futurePosition = getPose().getTranslation().plus(getFieldVelocity().times(time));
    
    Rotation2d currentTargetHeading = point.minus(currentPosition).getAngle().plus(rotationOffset);
    Rotation2d futureTargetHeading = point.minus(futurePosition).getAngle().plus(rotationOffset);
    
    double addedVelocity = futureTargetHeading.minus(currentTargetHeading).getRadians() / time;
    if (getPose().getTranslation().getDistance(point) < 1.0) {
      addedVelocity = 0.0;
    }


    setTargetHeadingAndVelocity(currentTargetHeading, addedVelocity);
  }


  /**
   * 
   * @return The target heading for the robot
   */
  public Rotation2d getTargetHeading() {
    return Rotation2d.fromRadians(alignmentController.getSetpoint());
  }

  public void setRotationTargetOverrideFromPoint(Supplier<Translation2d> point, Rotation2d rotationOffset) {
    rotationOverridePoint = point;
    rotationOverrideOffset = rotationOffset;
    addedAlignmentAngularVelocity = 0.0;
  }


  /**
   * This creates an "X" pattern with the wheels which makes the robot very hard to move
   */
  private void parkModules() {
    modules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    parked = true;
  }

  private void parkForAlignment() {
    modules[0].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
    modules[1].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[2].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    modules[3].setTargetState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Position to reset the odometer to
   */
  public void resetPose(Rotation2d heading, Pose2d pose, SwerveDriveWheelPositions wheelPositions) {
    poseEstimator.resetPosition(heading, wheelPositions, pose);
    alignmentController.setSetpoint(getHeading().getRadians());
  }

  /**
   * Resets the odometer position to a given position
   * @param pose Position to reset the odometer to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    alignmentController.setSetpoint(getHeading().getRadians());
  }

  public boolean canZeroHeading() {
    return (parked || isAligning || RobotState.isDisabled()) && (Math.abs(getRotationalVelocity()) < 0.5);
  }

  /**
   * 
   * @param visionMeasurement The robot position on the field from the apriltags
   */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp, Matrix<N3,N1> visionMeasurementStdDevs) {

    // System.out.println(visionMeasurement);
    // System.out.println(visionMeasurementStdDevs.get(0, 0));
    // System.out.println(visionMeasurementStdDevs.get(1, 0));
    // System.out.println(visionMeasurementStdDevs.get(2, 0));


    Rotation2d oldHeading = getHeading();
    poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    poseEstimator.addVisionMeasurement(visionMeasurement, timestamp);
    Rotation2d newHeading = getHeading();
    // Logger.log("newHeading.minus(oldHeading)", newHeading.minus(oldHeading).getRadians());
    // Logger.log("alignmentController.getSetpoint()", alignmentController.getSetpoint());
    // Logger.log("getHeading()", getHeading().getRadians());
    alignmentController.setSetpoint(Rotation2d.fromRadians(alignmentController.getSetpoint()).plus(newHeading).minus(oldHeading).getRadians());
  }

  /**
   * Stops all motors on all modules
   */
  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  public Translation2d getFieldVelocity() {
    ChassisSpeeds fieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeeds(), getHeading());
    return new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond);
  }

  public double getRotationalVelocity() {
    return getMeasuredChassisSpeeds().omegaRadiansPerSecond;
  }

  /**
   * @return Target chassis x, y, and rotational velocity (robot-relative)
   */
  private ChassisSpeeds getTargetChassisSpeeds() {
    return kinematics.toChassisSpeeds(getTargetModuleStates());
  }

  /**
   * @return Measured chassis x velocity, y velocity, and rotational velocity (robot-relative)
   */
  private ChassisSpeeds getMeasuredChassisSpeeds() {
    return kinematics.toChassisSpeeds(getMeasuredModuleStates());
  }

  /**
   * @return Driven chassis x speed, y speed, and rotational speed (robot-relative)
   */
  private ChassisSpeeds getDrivenChassisSpeeds() {
    return drivenChassisSpeeds;
  }

  /**
   * @return Measured module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    return positions;
  }

  /**
   * @return Target module states (speed and direction)
   */
  private SwerveModuleState[] getTargetModuleStates() {
    SwerveModuleState[] targetStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      targetStates[i] = modules[i].getTargetState();
    }
    return targetStates;
  }

  /**
   * @return Measured module states (speed and direction)
   */
  private SwerveModuleState[] getMeasuredModuleStates() {
    SwerveModuleState[] measuredStates = new SwerveModuleState[SWERVE_DRIVE.MODULE_COUNT];
    for (int i = 0; i < SWERVE_DRIVE.MODULE_COUNT; i++) {
      measuredStates[i] = modules[i].getMeasuredState();
    }
    return measuredStates;
  }
  
  /**
   * @return Total current through all modules
   */
  public double getCurrent() {
    double totalCurrent = 0.0;
    for (SwerveModule module : modules)
      totalCurrent += module.getTotalCurrent();
    return totalCurrent;
  }

  /**
   * @return This swerve drive's NavX AHRS IMU Gyro
   */
  public static AHRS getGyro() {
    return gyro;
  }

  /**
   * Resets gyro heading
   */
  public void resetGyroHeading(Rotation2d newHeading) {
    gyroOffset = newHeading.minus(gyroHeading);
    alignmentController.reset();
    alignmentController.setSetpoint(newHeading.getRadians());
  }

  /**
   * @return Gyro heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * @return Heading as a Rotation2d based on alliance
   */
  public Rotation2d getAllianceAwareHeading() {
    return getHeading().plus(Rotation2d.fromDegrees(Constants.IS_BLUE_TEAM.get() ? 0.0 : 180.0));
  }

  /**
   * @return Pose on the field from odometer data as a Pose2d
   */
  public Pose2d getPose() {
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
    return estimatedPose;
  }

  public Pose2d getPose(double timestampSeconds) {
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition(timestampSeconds);
    return estimatedPose;
  }

  public Pose2d getFuturePose() {
    Translation2d futurePosition = getPose().getTranslation();
    futurePosition = futurePosition.plus(getFieldVelocity().times(getFieldVelocity().getNorm()).div(2.0 * Constants.SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION));
    return new Pose2d(futurePosition, getPose().getRotation());
  }

  public boolean underStage() {
    return MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], getFuturePose().getTranslation()) ||
           MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], getFuturePose().getTranslation()) ||
           MathUtils.isInsideTriangle(Field.BLUE_STAGE_CORNERS[0], Field.BLUE_STAGE_CORNERS[1], Field.BLUE_STAGE_CORNERS[2], getPose().getTranslation()) ||
           MathUtils.isInsideTriangle(Field.RED_STAGE_CORNERS[0], Field.RED_STAGE_CORNERS[1], Field.RED_STAGE_CORNERS[2], getPose().getTranslation());
  }

  /**
   * @return Field2d object for SmartDashboard widget.
   */
  public static Field2d getField() {
    return field;
  }

  /**
   * Converts the speed of a wheel moving to the angular velocity of the robot as if it's
   * rotating in place
   * @param wheelSpeed Drive velocity in m/s
   * @return Equivalent rotational velocity in rad/s
   * @see #toLinear(double)
   */
  public static double toAngular(double wheelSpeed) {
    return wheelSpeed / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }

  /**
   * Converts the angular velocity of the robot to the speed of a wheel moving as if the
   * robot is rotating in place
   * @param angularVelocity Rotational velocity in rad/s
   * @return Equivalent drive velocity in m/s
   * @see #toAngular(double)
   */
  public static double toLinear(double angularVelocity) {
    return angularVelocity * SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS;
  }

  /**
   * Create a kinematics object for the swerve drive based on the SWERVE_DRIVE constants
   * @return A SwerveDriveKinematics object that models the swerve drive
  */
  public static SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d( SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, SWERVE_DRIVE.WHEELBASE  / 2.0), 
      new Translation2d(-SWERVE_DRIVE.TRACKWIDTH / 2.0, -SWERVE_DRIVE.WHEELBASE / 2.0));
  }

  public static void printChoreoConfig() {
    System.out.println(
      String.format(
        """


////////////////////////////////
///// CHOREO ROBOT CONFIG //////
////////////////////////////////

---------- DIMENSIONS ----------
  Mass: %.3f kg
  MOI: %.3f kg * m^2
  Bumper Width: %.3f m
  Bumper Length: %.3f m
  Wheelbase: %.3f m
  Trackwidth: %.3f m

--------- DRIVE MOTOR ----------
  Wheel Radius: %.3f m
  Gearing: %.3f : 1
  Motor Max Speed: %.0f RPM
  Motor Max Torque: %.3f N * m


        """,
        SWERVE_DRIVE.ROBOT_MASS,
        SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA,
        SWERVE_DRIVE.BUMPER_WIDTH,
        SWERVE_DRIVE.BUMPER_LENGTH,
        SWERVE_DRIVE.WHEELBASE,
        SWERVE_DRIVE.TRACKWIDTH,
        SWERVE_DRIVE.WHEEL_RADIUS,
        SWERVE_DRIVE.DRIVE_MOTOR_GEARING,
        SWERVE_DRIVE.PHYSICS.MAX_MOTOR_SPEED,
        SWERVE_DRIVE.PHYSICS.MAX_MOTOR_TORQUE
      )
    );
  }

  /**
   * Follow a choreo trajectory
   * @param pathName Name of the path to follow
   * @param first Whether or not this is the first command in autonomous mode
   * @return A command to run
   */
  public Command followChoreoTrajectory(String pathName, boolean first) {

    ChoreoTrajectory trajectory = Choreo.getTrajectory(pathName);

    field.getObject("traj").setPoses(
      trajectory.getInitialPose(), trajectory.getFinalPose()
    );
    field.getObject("trajPoses").setPoses(
      trajectory.getPoses()
    );

    Command swerveCommand = Choreo.choreoSwerveCommand(
        // Choreo trajectory to follow
        trajectory,

        // A supplier that returns the current field-relative pose of the robot based on the wheel
        // and vision odometry
        this::getPose,

        // PIDControllers for correcting errors in field-relative translation (input: X or Y error in
        // meters, output: m/s).
        new PIDController(
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, 
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD
        ),
        new PIDController(
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kI, 
          SWERVE_DRIVE.AUTONOMOUS.TRANSLATION_GAINS.kD
        ),

        // PIDController to correct for rotation error (input: heading error in radians, output: rad/s)
        new PIDController(
          SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kP,
          SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kI,
          SWERVE_DRIVE.AUTONOMOUS.ROTATION_GAINS.kD
        ),

        // A consumer which drives the robot in robot-relative coordinates
        this::driveRobotRelative,
        
        // A supplier which returns whether or not to mirror the path based on alliance (this assumes
        // the path is created for the blue alliance)
        () -> false,

        // The subsystem(s) to require, typically your drive subsystem only
        this
    );
    
    if (first) {
      return Commands.sequence(
        Commands.runOnce(() -> System.out.println("===== STARTING AUTO =====")),
        Commands.runOnce(() -> this.resetPose(trajectory.getInitialPose())),
        swerveCommand
      );
    } else {
      return Commands.sequence(
        Commands.runOnce(() -> System.out.println("===== STARTING AUTO =====")),
        swerveCommand
      );
    }
  }

  /**
   * Go to a position on the field
   * @param goalPosition Field-relative position on the field to go to
   * @param orientation Field-relative orientation to rotate to
   * @return A command to run
   */
  // public Command goTo(Pose2d pose) {
  //   return pathfindThenFollowPath(
  //     new Pose2d(new Translation2d(-1.0, 0).rotateBy(pose.getTranslation().minus(getPose().getTranslation()).getAngle()).plus(pose.getTranslation()), pose.getRotation()),
  //     pose
  //   );

  //   // Command pathfindingCommand = goToSimple(pose);

  //   // if (pose.getTranslation().getDistance(getPose().getTranslation()) > 1.0) {
  //   //   // Since AutoBuilder is configured, we can use it to build pathfinding commands
  //   //   pathfindingCommand = AutoBuilder.path(
  //   //     pose,
  //   //     SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
  //   //     0.0, // Goal end velocity in meters/sec
  //   //     0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  //   //   ).andThen(goToSimple(pose));
  //   // }

  //   // if (RobotState.isAutonomous()) {
  //   //   return pathfindingCommand;
  //   // } else {
  //   //   return pathfindingCommand.andThen(Commands.runOnce(() -> setTargetHeading(pose.getRotation())));
  //   // }
  // }

  public Command pathfindThenFollowPath(Pose2d firstPoint, Pose2d secondPoint) {
    Rotation2d angle = secondPoint.getTranslation().minus(firstPoint.getTranslation()).getAngle();

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(firstPoint.getTranslation(), angle),
      new Pose2d(secondPoint.getTranslation(), angle)
    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      new GoalEndState(
        0.0,
        secondPoint.getRotation(),
        true
      )
    );
    
    return AutoBuilder.pathfindThenFollowPath(
      path,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  /**
   * Go to a position on the field (without object avoidence)
   * @param pose Field-relative pose on the field to go to
   * @param xboxController Xbox controller to cancel the command
   * @return A command to run
   */
  public Command goToSimple(Pose2d pose) {
    Rotation2d angle = pose.getTranslation().minus(getPose().getTranslation()).getAngle();

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(getPose().getTranslation(), angle),
      new Pose2d(pose.getTranslation(), angle)
    );

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      SWERVE_DRIVE.AUTONOMOUS.DEFAULT_PATH_CONSTRAINTS,
      new GoalEndState(
        0.0,
        pose.getRotation(),
        true
      )
    );

    return Commands.sequence(
      AutoBuilder.followPath(path),
      runOnce(() -> setTargetHeading(pose.getRotation()))
    );
  }

  // /**
  //  * Go to the nearest pose in a list of poses
  //  * @param poses List of poses to go to
  //  * @param xboxController Xbox controller to cancel the command
  //  * @return A command to run
  //  */
  // public Command goToNearestPose(List<Pose2d> poses) {
  //  return goTo(getPose().nearest(poses));
  // }
  
  public boolean shouldFlipPaths() {
    return false;
  }
}