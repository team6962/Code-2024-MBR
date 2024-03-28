// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Constants.SHOOTER_PIVOT;
import frc.robot.Constants.Field;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.util.software.Logging.Logger;

public class Shooter extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private ShooterWheels shooterWheels;
  private ShooterPivot shooterPivot;

  private Mechanism2d mechanism = new Mechanism2d(0, 0);
  private MechanismRoot2d pivotMechanism = mechanism.getRoot("pivot", SHOOTER_PIVOT.POSITION.getX(), SHOOTER_PIVOT.POSITION.getZ());
  private MechanismLigament2d shooterMechanism = pivotMechanism.append(new MechanismLigament2d("shooter", SHOOTER_PIVOT.SHOOTER_LENGTH, .0));

  private Supplier<Translation3d> aimingPoint = Field.SPEAKER;
  private boolean isAiming = false;
  private double targetSize = 0.0;

  public static enum State {
    IN,
    AIM_SPEAKER,
    AIM_MORTAR,
    SPIN_UP,
  }

  public Shooter(SwerveDrive swerveDrive) {
    this.shooterWheels = new ShooterWheels();
    this.shooterPivot = new ShooterPivot();
    this.swerveDrive = swerveDrive;
    
    Logger.autoLog(this, "Is Aimed", () -> isAimed());
    Logger.autoLog(this, "Shooter Position", () -> ShooterMath.calcShooterLocationOnField(swerveDrive, this));
    Logger.autoLog(this, "Straight Line Angle", () -> {
      Translation3d loc = Field.SPEAKER.get().minus(ShooterMath.calcShooterLocationOnField(swerveDrive, this));
      return Rotation2d.fromRadians(Math.atan(loc.getZ() / loc.toTranslation2d().getNorm())).minus(Constants.SHOOTER_PIVOT.NOTE_ROTATION_OFFSET).getRadians();
    }
    );
    Logger.autoLog(this, "Speaker Distance", () -> ShooterMath.calcShooterLocationOnField(swerveDrive, this).getDistance(Field.SPEAKER.get()));
    Logger.autoLog(this, "Speaker Height", () -> ShooterMath.calcShooterLocationOnField(swerveDrive, this).minus(Field.SPEAKER.get()).getZ());
    Logger.autoLog(this, "Speaker Floor Distance", () -> ShooterMath.calcShooterLocationOnField(swerveDrive, this).toTranslation2d().getDistance(Field.SPEAKER.get().toTranslation2d()));
    Logger.autoLog(this, "Shot Heading", () -> ShooterMath.calcVelocityCompensatedPoint(
          Field.SPEAKER.get(),
          swerveDrive,
          this
        ).toTranslation2d().minus(swerveDrive.getPose().getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0)).getDegrees());
    Logger.autoLog(this, "Aiming Point", () -> ShooterMath.calcVelocityCompensatedPoint(
      Field.SPEAKER.get(),
      swerveDrive,
      this
    ));
    SmartDashboard.putData("ShooterMechanism", mechanism);
  }
  

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_SHOOTER) return;
    shooterMechanism.setAngle(Rotation2d.fromDegrees(180.0).minus(shooterPivot.getPosition()));
    if (RobotState.isAutonomous()) {
      shooterPivot.setTargetAngle(
        ShooterMath.calcPivotAngle(
          ShooterMath.calcVelocityCompensatedPoint(Field.SPEAKER.get(), swerveDrive, this),
          swerveDrive,
          this
        )
      );
    }

    Translation3d compensatedAimingPoint = ShooterMath.calcVelocityCompensatedPoint(aimingPoint.get(), swerveDrive, this);
    
    Rotation2d pivotAngle = ShooterMath.calcPivotAngle(compensatedAimingPoint, swerveDrive, this);
    double projectileVelocity = shooterPivot.isAngleAchievable(ShooterMath.calcPivotAngle(aimingPoint.get(), swerveDrive, this, Constants.SHOOTER_WHEELS.MAX_WHEEL_SPEED)) ? Constants.SHOOTER_WHEELS.TOP_EXIT_VELOCITY : ShooterMath.calcProjectileVelocity(
      compensatedAimingPoint,
      swerveDrive,
      this
    );
    double flightTime = ShooterMath.calculateFlightTime(compensatedAimingPoint, swerveDrive, this);
    double shooterWheelVelocity = ShooterMath.calcShooterWheelVelocity(projectileVelocity);
    
    Logger.log("compensatedAimingPoint", compensatedAimingPoint);
    Logger.log("targetPivotAngle", pivotAngle == null ? 0.0 : pivotAngle.getDegrees());
    Logger.log("realPivotAngle", shooterPivot.getPosition().getDegrees());
    Logger.log("flightTime", flightTime);
    Logger.log("targetShooterWheelVelocity", shooterWheelVelocity);
    Logger.log("realShooterWheelVelocity", shooterWheels.getVelocity());
    Logger.log("targetProjectileVelocity", projectileVelocity);
    Logger.log("realProjectileVelocity", ShooterMath.calcProjectileVelocity(shooterWheels.getVelocity()));


    // System.out.println(ShooterMath.calcProjectileVelocity(
    //           ShooterMath.calcVelocityCompensatedPoint(
    //             aimingPoint.get(),
    //             swerveDrive,
    //             this
    //           ),
    //           swerveDrive,
    //           this
    //         ));

    // System.out.println(ShooterMath.calcProjectileVelocity(getWheels().getVelocity()));
  }

  public Command setState(State state) {
    switch(state) {
      case AIM_SPEAKER:
        return Commands.parallel(
          aim(Field.SPEAKER, Field.SPEAKER_HEIGHT)
        );
      case AIM_MORTAR:
        return Commands.parallel(
          aim(Field.MORTAR_POINT, 10.0).finallyDo(
            () -> {
              this.aimingPoint = Field.SPEAKER;
              this.targetSize = Field.SPEAKER_HEIGHT;
            }
          )
        );
      case SPIN_UP:
        return Commands.parallel(
          shooterWheels.setState(ShooterWheels.State.SPIN_UP),
          shooterWheels.setTargetExitVelocityCommand(() -> 
            shooterPivot.isAngleAchievable(ShooterMath.calcPivotAngle(aimingPoint.get(), swerveDrive, this, Constants.SHOOTER_WHEELS.MAX_WHEEL_SPEED)) ? Constants.SHOOTER_WHEELS.TOP_EXIT_VELOCITY : ShooterMath.calcProjectileVelocity(
              ShooterMath.calcVelocityCompensatedPoint(
                aimingPoint.get(),
                swerveDrive,
                this
              ),
              swerveDrive,
              this
            )
          )
      );
    } 
    return null;
  }

  public ShooterWheels getWheels() {
    return shooterWheels;
  }

  public Command aim(Supplier<Translation3d> point, double targetSize) {
    // Calculate point to aim towards, accounting for current velocity
    return shooterPivot.setTargetAngleCommand(() -> 
      ShooterMath.calcPivotAngle(
        ShooterMath.calcVelocityCompensatedPoint(
          point.get(),
          swerveDrive,
          this
        ),
        swerveDrive,
        this
      )
    ).alongWith(
      swerveDrive.facePointCommand(() -> 
        ShooterMath.calcVelocityCompensatedPoint(
          point.get(),
          swerveDrive,
          this
        ).toTranslation2d(),
        Rotation2d.fromDegrees(180.0)
      )
    ).alongWith(
      Commands.runOnce(() -> {
        this.aimingPoint = point;
        this.targetSize = targetSize;
        this.isAiming = true;
      })
    ).finallyDo(() -> this.isAiming = false);
  }

  public boolean doneMoving() {
    return shooterPivot.doneMoving();
  }

  public boolean isAimed() {
    return ShooterMath.isAimed(
      aimingPoint.get(),
      targetSize,
      swerveDrive,
      this
    );
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }

  public ShooterPivot getPivot() {
    return shooterPivot;
  }

  public boolean inRange() {
    return ShooterMath.inRange(aimingPoint.get(), swerveDrive, this);
  }

  public boolean isAiming() {
    return isAiming;
  }
}
