package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.SHOOTER_PIVOT;
import frc.robot.Constants.Constants.SHOOTER_WHEELS;
import frc.robot.Constants.Preferences;
import frc.robot.subsystems.drive.SwerveDrive;

public class ShooterMath {
  
  /**
   * 
   * @param targetPoint 3D position of target point on the field
   * @param currentPose Current swerve drive pose
   * @param shooterWheelVelocity shooter wheel velocity in rad/s
   * @return
   */
  
  
  // gets the position you should shoot from, using the given shooter angle
  public static double calcShootingDistance(Translation3d targetPoint, double shooterWheelVelocity, Rotation2d pivotAngle){
    // vertical distance between release point and target
    double targetHeight = targetPoint.getZ() - SHOOTER_PIVOT.POSITION.getZ(); 
    double projectileVelocity = calcProjectileVelocity(shooterWheelVelocity);
    double gravity = 9.80;
    double shootingDistance;
    
    try {
      shootingDistance = (
        ((Math.pow(projectileVelocity, 2.0) * Math.tan(pivotAngle.getDegrees())) / gravity) - 
        ((Math.sqrt(-2.0 * gravity * targetHeight * (Math.pow(projectileVelocity, 2.0)) * 
        Math.pow(Math.tan(pivotAngle.getDegrees()), 2.0)) + (Math.pow(projectileVelocity, 4.0) * 
        Math.pow(Math.tan(pivotAngle.getDegrees()), 2.0)) - (2.0 * gravity * targetHeight * 
        Math.pow(projectileVelocity, 2.0))) / 2.0)) / (Math.pow(Math.tan(pivotAngle.getDegrees()), 2.0) + 1.0);
    } catch (Exception e){
      return 0.0;
    }

    //todo: maybe return a band of pose2ds
    return shootingDistance;
  }

  public static boolean inRange(Translation3d targetPoint, SwerveDrive swerveDrive, Shooter shooter) {
    Rotation2d pivotAngle = calcPivotAngle(targetPoint, swerveDrive, shooter, shooter.getWheels().getVelocity());
    if (pivotAngle == null) return false;
    // System.out.println(pivotAngle);
    return shooter.getPivot().isAngleAchievable(pivotAngle);
  }

  public static double calcProjectileVelocity(Translation3d targetPoint, SwerveDrive swerveDrive, Shooter shooter) {
    Rotation2d pivotAngle = shooter.getPivot().getPosition();

    pivotAngle = Rotation2d.fromDegrees(Math.min(Math.max(pivotAngle.getDegrees(), Preferences.SHOOTER_PIVOT.MIN_ANGLE.getDegrees() + 10.0), Preferences.SHOOTER_PIVOT.MAX_ANGLE.getDegrees() - 10.0));

    Rotation2d exitAngle = pivotAngle.plus(Constants.SHOOTER_PIVOT.NOTE_ROTATION_OFFSET);

    Translation3d shooterLocation = calcShooterLocationOnField(swerveDrive, shooter);
    double targetHeight = targetPoint.getZ() - shooterLocation.getZ();
    if (targetHeight > 0) return Constants.SHOOTER_WHEELS.TOP_EXIT_VELOCITY;
    double floorDistance = shooterLocation.toTranslation2d().getDistance(targetPoint.toTranslation2d());
    double gravity = 9.80;
    double velocity = (Math.sqrt(floorDistance) * Math.sqrt(gravity) * Math.sqrt(Math.pow(Math.tan(exitAngle.getRadians()), 2.0) + 1)) / Math.sqrt(2.0 * Math.tan(exitAngle.getRadians()) - (2.0 * targetHeight / floorDistance));
    
    if (Double.isNaN(velocity)) velocity = Constants.SHOOTER_WHEELS.TOP_EXIT_VELOCITY;
    return velocity;
  }

  public static Rotation2d calcPivotAngle(Translation3d targetPoint, SwerveDrive swerveDrive, Shooter shooter, double shooterWheelVelocity) {    
    if (Math.abs(shooterWheelVelocity) < 1.0) return null;
    
    Rotation2d pivotAngle = Rotation2d.fromDegrees(0);
    int iterations = 4;

    for (int i = 0; i < iterations; i++) {
      Translation3d shooterLocation = calcShooterLocationOnField(swerveDrive, shooter);
      boolean mortarMode = targetPoint.getZ() < shooterLocation.getZ();

      double targetHeight = targetPoint.getZ() - shooterLocation.getZ();
      double floorDistance = shooterLocation.toTranslation2d().getDistance(targetPoint.toTranslation2d());
      double projectileVelocity = calcProjectileVelocity(shooterWheelVelocity);
      double gravity = 9.80;

      double exitRadians = Math.atan((Math.pow(projectileVelocity, 2.0) + (mortarMode ? 1 : -1) * Math.sqrt(Math.pow(projectileVelocity, 4.0) - gravity * (gravity * Math.pow(floorDistance, 2.0) + 2.0 * targetHeight * Math.pow(projectileVelocity, 2.0)))) / (gravity * floorDistance));
      double distanceAtApex = projectileVelocity * Math.cos(exitRadians) * (projectileVelocity * (Math.sin(exitRadians) / gravity));
      if (Double.isNaN(exitRadians) || (distanceAtApex < floorDistance && !mortarMode)) {
        return null;
      }
      Rotation2d exitAngle = Rotation2d.fromRadians(exitRadians);
      pivotAngle = exitAngle.minus(SHOOTER_PIVOT.NOTE_ROTATION_OFFSET);

      // System.out.println("pivotAngle " + pivotAngle.getDegrees());
      // System.out.println("targetHeight " + targetHeight);
      // System.out.println("floorDistance " + floorDistance);
      // System.out.println("projectileVelocity " + projectileVelocity);
    }

    return pivotAngle;
  }

  public static Rotation2d calcPivotAngle(Translation3d targetPoint, SwerveDrive swerveDrive, Shooter shooter) {
    Rotation2d realPivot = calcPivotAngle(targetPoint, swerveDrive, shooter, shooter.getWheels().getVelocity());
    if (realPivot == null) {
      return calcPivotAngle(targetPoint, swerveDrive, shooter, shooter.getWheels().getTargetVelocity());
    }
    return realPivot;
  }

  /**
   * 
   * @param shooterWheelVelocity shooter wheel velocity in rad/s
   * @return Projectile exit velocity in m/s
   */
  public static double calcProjectileVelocity(double shooterWheelVelocity) {
    if (shooterWheelVelocity >= Constants.SHOOTER_WHEELS.MAX_WHEEL_SPEED) return SHOOTER_WHEELS.MAX_EXIT_VELOCITY;
    double velocity = -Math.pow(SHOOTER_WHEELS.NOTE_LOG_BASE, -shooterWheelVelocity + SHOOTER_WHEELS.NOTE_LOG_OFFSET) + SHOOTER_WHEELS.MAX_EXIT_VELOCITY;
    if (velocity < 0) return 0.0;
    return velocity;

    // // Derived from https://www.reca.lc/shooterWheel
    // // return 1000000.0;
    // if (RobotState.isAutonomous()) {
    //   return SHOOTER_WHEELS.MAX_EXIT_VELOCITY;
    // }
    // return SHOOTER_WHEELS.MAX_EXIT_VELOCITY * (shooterWheelVelocity / (NEO.STATS.freeSpeedRadPerSec * SHOOTER_WHEELS.GEARBOX_STEP_UP));
    
    // return (shooterWheelVelocity * (SHOOTER_WHEELS.WHEEL_RADIUS + (Field.NOTE_THICKNESS - SHOOTER_WHEELS.COMPRESSION) / 2.0)) / 4.0;
    // double shooterWheelSurfaceSpeed = shooterWheelVelocity * SHOOTER_WHEELS.WHEEL_RADIUS;

    // double speedTransferPercentage = (SHOOTER_WHEELS.TOTAL_MOI * 20.0) / (SHOOTER_WHEELS.PROJECTILE_MASS * SHOOTER_WHEELS.WHEEL_RADIUS * 2.0 * SHOOTER_WHEELS.WHEEL_RADIUS * 2.0 * 7.0 + SHOOTER_WHEELS.TOTAL_MOI * 40.0);
    // return shooterWheelSurfaceSpeed * speedTransferPercentage;
  }

  public static double logB(double number, double custom_base) {
    // a is value and b is base
    double result = (int) (Math.log(number) / Math.log(custom_base));
    return result;
  }

  
  public static double calcShooterWheelVelocity(double projectileVelocity) {
    if (projectileVelocity >= SHOOTER_WHEELS.MAX_EXIT_VELOCITY) return Constants.SHOOTER_WHEELS.MAX_WHEEL_SPEED;
    return -logB(-projectileVelocity + SHOOTER_WHEELS.MAX_EXIT_VELOCITY, SHOOTER_WHEELS.NOTE_LOG_BASE) + SHOOTER_WHEELS.NOTE_LOG_OFFSET;
  }

  public static boolean isAimed(Translation3d targetPoint, double targetSize, SwerveDrive swerveDrive, Shooter shooter) {
    Translation3d shooterLocation = calcShooterLocationOnField(swerveDrive, shooter);
    Translation3d aimingPoint = calcVelocityCompensatedPoint(targetPoint, swerveDrive, shooter, true);
    double acceptableError = Math.atan(targetSize / (targetPoint.getDistance(shooterLocation) * 2.0));
    Rotation2d idealHeading = aimingPoint.toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0));

    Rotation2d idealPivotAngle = calcPivotAngle(aimingPoint, swerveDrive, shooter, shooter.getWheels().getVelocity());
    if (targetSize >= 10.0) idealPivotAngle = calcPivotAngle(aimingPoint, swerveDrive, shooter);
    if (idealPivotAngle == null) return false;
    // System.out.println(inRange(aimingPoint, swerveDrive, shooter));
    
    if (Math.abs(swerveDrive.getHeading().minus(idealHeading).getRadians()) > acceptableError * 2.0) return false;

    if (targetSize >= 10.0) return true;

    if (Math.abs(shooter.getPivot().getPosition().minus(idealPivotAngle).getRadians()) > acceptableError / 3.0) return false;
    if (!inRange(aimingPoint, swerveDrive, shooter)) return false;
    return true;
  }

  // public static double calcShotChance(Translation3d targetPoint, Pose2d currentPose, Translation2d currentVelocity, double rotationalVelocity, Rotation2d measuredPivotAngle, double shooterWheelVelocity) {
  //   Translation3d shooterLocation = calcShooterLocationOnField(currentPose, measuredPivotAngle);
  //   Translation3d aimingPoint = calcVelocityCompensatedPoint(targetPoint, currentPose, currentVelocity, rotationalVelocity, shooterWheelVelocity, measuredPivotAngle);

  //   double totalDistance = targetPoint.getDistance(shooterLocation);
    
  //   Translation3d speakerOffset = new Translation3d();

  //   if (Constants.IS_BLUE_TEAM.get()) {
  //     speakerOffset = new Translation3d(
  //       Math.cos(Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0,
  //       0.0,
  //       Math.sin(Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0
  //     );
  //   } else {
  //     speakerOffset = new Translation3d(
  //       -Math.cos(-Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0,
  //       0.0,
  //       -Math.sin(-Field.SPEAKER_ANGLE) * (Field.SPEAKER_HEIGHT - Field.NOTE_THICKNESS) / 2.0
  //     );
  //   }

  //   Translation3d minAimingPoint = aimingPoint.minus(speakerOffset);
  //   Translation3d maxAimingPoint = aimingPoint.plus(speakerOffset);

  //   Rotation2d minPivotAngle = calcPivotAngle(minAimingPoint, currentPose, shooterWheelVelocity);
  //   Rotation2d maxPivotAngle = calcPivotAngle(maxAimingPoint, currentPose, shooterWheelVelocity);

  //   double speakerVerticalDegreesOfView = maxPivotAngle.minus(minPivotAngle).getDegrees();
  //   double speakerLateralDegreesOfView = Units.radiansToDegrees(2.0 * Math.atan((Field.SPEAKER_WIDTH - Field.NOTE_LENGTH) / 2.0 / totalDistance));
    
  //   System.out.println("speakerVerticalDegreesOfView: " + speakerVerticalDegreesOfView);
  //   System.out.println("speakerLateralDegreesOfView: " +  speakerLateralDegreesOfView);

  //   System.out.println(40.0 / targetPoint.getDistance(new Translation3d(currentPose.getX(), currentPose.getY(), 0.0)));

  //   double velocityErrorDegrees = Math.abs(calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity + SHOOTER_WHEELS.SPEED_PRECISION).minus(calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity - SHOOTER_WHEELS.SPEED_PRECISION)).getDegrees());
    
  //   double verticalDegreesOfAccuracy = SHOOTER_PIVOT.ANGLE_PRECISION.getDegrees() * 2.0 + velocityErrorDegrees;
  //   double lateralDegreesOfAccuracy  = SHOOTER_PIVOT.HEADING_PRECISION.getDegrees() * 2.0;
    
  //   Rotation2d idealHeading = aimingPoint.toTranslation2d().minus(currentPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0));

  //   Rotation2d idealPivotAngle = calcPivotAngle(aimingPoint, currentPose, shooterWheelVelocity);
    
  //   double verticalErrorDegrees = Math.abs(idealPivotAngle.minus(measuredPivotAngle).getDegrees());
  //   double lateralErrorDegrees = Math.abs(idealHeading.minus(currentPose.getRotation()).getDegrees());
    
  //   double verticalValidShotDegrees = Math.min((speakerVerticalDegreesOfView / 2.0 + verticalDegreesOfAccuracy / 2.0) - verticalErrorDegrees, Math.min(speakerVerticalDegreesOfView, verticalDegreesOfAccuracy));
  //   double lateralValidShotDegrees = Math.min((speakerLateralDegreesOfView / 2.0 + lateralDegreesOfAccuracy / 2.0) - lateralErrorDegrees, Math.min(speakerLateralDegreesOfView, lateralDegreesOfAccuracy));
  //   verticalValidShotDegrees = Math.max(0, verticalValidShotDegrees);
  //   lateralValidShotDegrees = Math.max(0, lateralValidShotDegrees);
   
  //   double shotChance = (verticalValidShotDegrees * lateralValidShotDegrees) / (verticalDegreesOfAccuracy * lateralDegreesOfAccuracy);
    
  //   // if (idealPivotAngle.getDegrees() < Preferences.SHOOTER_PIVOT.MAX_ANGLE.getDegrees() && idealPivotAngle.getDegrees() > Preferences.SHOOTER_PIVOT.MIN_ANGLE.getDegrees() && shotChance == 1.0) {
  //   //   List<Pose2d> possibleShotPositions = SwerveDrive.getField().getObject("possibleShotPositions").getPoses();
  //   //   possibleShotPositions.add(currentPose);
  //   //   SwerveDrive.getField().getObject("possibleShotPositions").setPoses(possibleShotPositions);
  //   // }

  //   return shotChance;
  // }

  public static Translation3d calcShooterLocationOnField(SwerveDrive swerveDrive, Shooter shooter) {
    Pose2d currentPose = swerveDrive.getPose();
    Rotation2d pivotAngle = shooter.getPivot().getPosition();

    Translation2d swerveDrivePosition = currentPose.getTranslation();
    Translation3d shooterPosition = SHOOTER_PIVOT.POSITION;
    shooterPosition = shooterPosition.plus(new Translation3d(
      -SHOOTER_PIVOT.SHOOTER_LENGTH * Math.cos(pivotAngle.getRadians()),
      0.0,
      SHOOTER_PIVOT.SHOOTER_LENGTH * Math.sin(pivotAngle.getRadians())
    ));
    Translation3d shooterPositionRotated = shooterPosition.rotateBy(new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians()));
    return new Translation3d(shooterPositionRotated.getX() + swerveDrivePosition.getX(), shooterPositionRotated.getY() + swerveDrivePosition.getY(), shooterPositionRotated.getZ());
  }



  public static double calculateFlightTime(Translation3d targetPoint, SwerveDrive swerveDrive, Shooter shooter) {
    double shooterWheelVelocity = shooter.getWheels().getVelocity();
    if (Math.abs(shooterWheelVelocity) < 1.0) return 0.0;
    Rotation2d pivotAngle = shooter.getPivot().getPosition();
    // (v * sin(a) - sqrt(v^2 * sin(a)^2 - 2 * g * h)) / g
    double projectileVelocity = calcProjectileVelocity(shooterWheelVelocity);
    Rotation2d exitAngle = pivotAngle.plus(SHOOTER_PIVOT.NOTE_ROTATION_OFFSET);
    // System.out.println(targetPoint.toTranslation2d().getDistance(calcShooterLocationOnField(swerveDrive, shooter).toTranslation2d()) / (projectileVelocity * Math.cos(exitAngle.getRadians())));
    return targetPoint.toTranslation2d().getDistance(calcShooterLocationOnField(swerveDrive, shooter).toTranslation2d()) / (projectileVelocity * Math.cos(exitAngle.getRadians()));
  }

  public static Translation3d calcVelocityCompensatedPoint(Translation3d targetPoint, SwerveDrive swerveDrive, Shooter shooter) {
    return calcVelocityCompensatedPoint(targetPoint, swerveDrive, shooter, false);
  }

  public static Translation3d calcVelocityCompensatedPoint(Translation3d targetPoint, SwerveDrive swerveDrive, Shooter shooter, boolean toCheckAiming) {
    if (RobotState.isAutonomous() && !toCheckAiming) {
      return calcFutureAimingPoint(targetPoint, swerveDrive, shooter);
    }
    
    double shooterWheelVelocity = shooter.getWheels().getVelocity();
    Translation2d currentVelocity = swerveDrive.getFieldVelocity();
    
    if (Math.abs(shooterWheelVelocity) < 1.0) return targetPoint;
    
    double flightTime = calculateFlightTime(targetPoint, swerveDrive, shooter);

    if (Double.isNaN(flightTime) || Double.isInfinite(flightTime)) return targetPoint;
    
    Translation2d projectileOffset = currentVelocity.times(flightTime);

    if (toCheckAiming) {
      // Translation2d relativeShooterPosition = calcShooterLocationOnField(swerveDrive, shooter).toTranslation2d().minus(swerveDrive.getPose().getTranslation());
      // Translation2d rotationAddedOffset = relativeShooterPosition.div(relativeShooterPosition.getNorm()).times(swerveDrive.getRotationalVelocity() * relativeShooterPosition.getNorm() * flightTime).rotateBy(Rotation2d.fromDegrees(90.0));
      // projectileOffset = projectileOffset.minus(rotationAddedOffset);
    }

    Translation3d velocityCompensatedPoint =  new Translation3d(
      targetPoint.getX() - projectileOffset.getX(),
      targetPoint.getY() - projectileOffset.getY(),
      targetPoint.getZ()
    );
    
    return velocityCompensatedPoint;
  }

  public static Translation3d calcFutureAimingPoint(Translation3d targetPoint, SwerveDrive swerveDrive, Shooter shooter) {
    Translation2d futureOffset = swerveDrive.getPose().getTranslation().minus(swerveDrive.getFuturePose().getTranslation());
    Translation3d futureAimingPoint =  new Translation3d(
      targetPoint.getX() + futureOffset.getX(),
      targetPoint.getY() + futureOffset.getY(),
      targetPoint.getZ()
    );
    SwerveDrive.getField().getObject("futureAimingPoint").setPose(new Pose2d(futureAimingPoint.toTranslation2d(), new Rotation2d()));
    
    return futureAimingPoint;
  }

}
