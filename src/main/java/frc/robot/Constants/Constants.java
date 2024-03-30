// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final Supplier<Boolean> IS_BLUE_TEAM = () -> !(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

  // ENABLED SYSTEMS
  public static final class ENABLED_SYSTEMS {
    public static final boolean ENABLE_DRIVE     = true;
    public static final boolean ENABLE_DASHBOARD = true;
    public static final boolean ENABLE_SHOOTER   = true;
    public static final boolean ENABLE_INTAKE   = true;
    public static final boolean ENABLE_TRANSFER = true;
    public static final boolean ENABLE_HANG = false;
    public static final boolean ENABLE_AMP = true;
  }

  public static final class LOGGING {
    public static final int LOGGING_PERIOD_MS = 20;
  }

  // DEVICES
  public static final class DEVICES {
    public static final int DRIVE_XBOX_CONTROLLER = 0;
    public static final int OPERATOR_XBOX_CONTROLLER = 1;
  }

  // DASHBOARD (ShuffleBoard)
  public static final class DASHBOARD {
    public static final String TAB_NAME = "SwerveDrive";
  }

  // LIMELIGHT
  // Exposure: 750
  // Sensor Gain: 10
  public static final class LIMELIGHT {
    // x is front-to-back
    // y is left-to-right
    // z it top-to-bottom
    public static final Map<String, Pose3d> APRILTAG_CAMERA_POSES = Map.of(
      "limelight-ftag", new Pose3d(Units.inchesToMeters(7.442142), Units.inchesToMeters(1.0), Units.inchesToMeters(25.283), new Rotation3d(0.0, Units.degreesToRadians(24.0), 0.0)),
      "limelight-btag", new Pose3d(Units.inchesToMeters(2.670592), Units.inchesToMeters(-3.0), Units.inchesToMeters(25.283), new Rotation3d(0.0, Units.degreesToRadians(24.0), Units.degreesToRadians(180.0)))
    );

    public static final String NOTE_CAMERA_NAME = "limelight-fnote";
    public static final int[] BLACKLISTED_APRILTAGS = {};

    public static final Rotation2d NOTE_CAMERA_PITCH = Rotation2d.fromDegrees(-24);
    // x is forward, y is left, z is up
    public static final Translation3d NOTE_CAMERA_POSITION = new Translation3d(Units.inchesToMeters(13.0), Units.inchesToMeters(0.0), Units.inchesToMeters(22.5));

    public static final Rotation2d FOV_HEIGHT = Rotation2d.fromDegrees(48.9); // Degrees
    public static final Rotation2d FOV_WIDTH = Rotation2d.fromDegrees(62.5); // Degrees
    public static final double NOTE_CAMERA_HEIGHT_PIXELS = 960;
  }

  // SWERVE DRIVE
  public static final class SWERVE_DRIVE {

    ///////////////////////// CONFIG /////////////////////////
    public static final boolean  IS_PROTOTYPE_CHASSIS               = !new DigitalInput(0).get();

    public static final double   INSPECTION_WEIGHT                  = IS_PROTOTYPE_CHASSIS ? Units.lbsToKilograms(42) : Units.lbsToKilograms(113.5);
    public static final double   BATTERY_WEIGHT                     = Units.lbsToKilograms(12.89);
    public static final double   BUMPER_WEIGHT                      = IS_PROTOTYPE_CHASSIS ? 0.0 : Units.lbsToKilograms(11.0);
    public static final double   ROBOT_MASS                         = INSPECTION_WEIGHT + BATTERY_WEIGHT + BUMPER_WEIGHT; // kg
    public static final double   FRICTION_COEFFICIENT               = 1.0;
    public static final int      MODULE_COUNT                       = 4;
    public static final double   CHASSIS_WIDTH                      = Units.inchesToMeters(28);
    public static final double   CHASSIS_LENGTH                     = Units.inchesToMeters(28);
    public static final double   BUMPER_THICKNESS                   = Units.inchesToMeters(3.25);
    public static final double   WHEEL_TO_EDGE_DISTANCE             = Units.inchesToMeters(2.625);
    public static final double   WHEEL_RADIUS                       = Units.inchesToMeters(2.0);
    public static final double   WHEEL_WIDTH                        = Units.inchesToMeters(2.0);
    public static final double   DRIVE_MOTOR_GEARING                = 6.75;
    public static final double   STEER_MOTOR_GEARING                = 150.0 / 7.0;
    public static final double   GEARBOX_EFFICIENCY                 = 0.975;
    public static final double   BATTERY_RESISTANCE                 = 0.02; // ohms
    public static final double   BATTERY_VOLTAGE                    = 12.6; // volts
    public static final double   BROWNOUT_VOLTAGE                   = 6.8; // volts

    // public static final double   VELOCITY_DEADBAND                  = 0.05; // Velocity we stop moving at
    
    // ODOMETER
    public static final Supplier<Pose2d>   STARTING_POSE            = Field.pose2d(0.0, 0.0, 0.0);

    // TESTING
    public static final double   MOTOR_POWER_HARD_CAP               = 1.0; // Only use for testing, otherwise set to 1.0
    
    // REDUCE DRIVE VELOCITY WHEN FAR FROM ANGLE
    public static final boolean  DO_ANGLE_ERROR_SPEED_REDUCTION     = false;
    

    ///////////////////////// CALCUALTED /////////////////////////

    // PHYSICAL
    public static final double   TRACKWIDTH                         = CHASSIS_WIDTH - WHEEL_TO_EDGE_DISTANCE * 2.0; // left-to-right distance between the drivetrain wheels
    public static final double   WHEELBASE                          = CHASSIS_LENGTH - WHEEL_TO_EDGE_DISTANCE * 2.0; // front-to-back distance between the drivetrain wheels
    public static final double   BUMPER_WIDTH                       = SWERVE_DRIVE.CHASSIS_WIDTH + SWERVE_DRIVE.BUMPER_THICKNESS * 2.0;
    public static final double   BUMPER_LENGTH                      = SWERVE_DRIVE.CHASSIS_LENGTH + SWERVE_DRIVE.BUMPER_THICKNESS * 2.0;
    public static final double   BUMPER_DIAGONAL                    = Math.hypot(SWERVE_DRIVE.BUMPER_WIDTH, SWERVE_DRIVE.BUMPER_LENGTH + Units.inchesToMeters(3.5 * 2.0));
    public static final double   MAX_CURRENT_DRAW                   = (BATTERY_VOLTAGE - BROWNOUT_VOLTAGE) / BATTERY_RESISTANCE;

    
    // GEAR AND WHEEL RATIOS
    public static final double   DRIVE_ENCODER_CONVERSION_FACTOR    = (WHEEL_RADIUS * 2.0 * Math.PI) / DRIVE_MOTOR_GEARING;
    public static final double   STEER_ENCODER_CONVERSION_FACTOR    = (Math.PI * 2.0) / STEER_MOTOR_GEARING;
    
    public static class PHYSICS {
      public static final double ROTATIONAL_INERTIA                 = 1.25 * ((1.0 / 12.0) * ROBOT_MASS * (Math.pow(BUMPER_WIDTH, 2.0) + Math.pow(BUMPER_LENGTH, 2.0)));
      public static final double SLIPLESS_ACCELERATION              = 9.80 * FRICTION_COEFFICIENT;
      public static final int    SLIPLESS_CURRENT_LIMIT             = (int) ((SLIPLESS_ACCELERATION * NEO.STATS.stallCurrentAmps * ROBOT_MASS * WHEEL_RADIUS) / (4.0 * DRIVE_MOTOR_GEARING * NEO.STATS.stallTorqueNewtonMeters));
      
      public static final double MAX_MOTOR_SPEED                    = Units.radiansPerSecondToRotationsPerMinute(NEO.STATS.freeSpeedRadPerSec) * GEARBOX_EFFICIENCY;
      public static final double MAX_MOTOR_TORQUE                   = NEO.maxTorqueCurrentLimited(SLIPLESS_CURRENT_LIMIT);
      
      public static final double MAX_WHEEL_VELOCITY                 = (MAX_MOTOR_SPEED * (Math.PI * 2.0)) / 60.0 / DRIVE_MOTOR_GEARING;
      
      public static final double MAX_LINEAR_VELOCITY                = MAX_WHEEL_VELOCITY * WHEEL_RADIUS;
      public static final double MAX_LINEAR_FORCE                   = (4.0 * MAX_MOTOR_TORQUE * DRIVE_MOTOR_GEARING) / WHEEL_RADIUS;
      public static final double MAX_LINEAR_ACCELERATION            = MAX_LINEAR_FORCE / ROBOT_MASS;
      
      public static final double DRIVE_RADIUS                       = Math.hypot(WHEELBASE / 2.0, TRACKWIDTH / 2.0);
      public static final double MAX_ANGULAR_TORQUE                 = MAX_LINEAR_FORCE * DRIVE_RADIUS;
      public static final double MAX_ANGULAR_ACCELERATION           = MAX_ANGULAR_TORQUE / ROTATIONAL_INERTIA; // TODO FIGURE OUT WHY WRONG
      public static final double MAX_ANGULAR_VELOCITY               = (MAX_WHEEL_VELOCITY * WHEEL_RADIUS) / DRIVE_RADIUS;
    }

    // Used only for when we have errors in the path (aka only when wheels slip or we're bumped off course)
    public static final class AUTONOMOUS {
      public static final class TRANSLATION_GAINS {
        public static final double kP = 2.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }
      public static final class ROTATION_GAINS {
        public static final double kP = 2.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
      }

      public static final double ACCELERATION_REDUCTION = ((SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION * SWERVE_DRIVE.ROBOT_MASS + ((SWERVE_DRIVE.PHYSICS.ROTATIONAL_INERTIA * SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION) / SWERVE_DRIVE.PHYSICS.DRIVE_RADIUS)) / (9.80 * SWERVE_DRIVE.ROBOT_MASS * SWERVE_DRIVE.FRICTION_COEFFICIENT));

      public static final PathConstraints DEFAULT_PATH_CONSTRAINTS =
        new PathConstraints(
          SWERVE_DRIVE.PHYSICS.MAX_LINEAR_VELOCITY,
          SWERVE_DRIVE.PHYSICS.MAX_LINEAR_ACCELERATION / ACCELERATION_REDUCTION,
          SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_VELOCITY,
          SWERVE_DRIVE.PHYSICS.MAX_ANGULAR_ACCELERATION / ACCELERATION_REDUCTION
        );
    }

    public static final class DRIVE_MOTOR_PROFILE {
      // FROM WPILIB SYSTEM IDENTIFICATION, FREE SPINNING
      public static final double kP                 = 0.00500; // Proportion Gain
      public static final double kI                 = 0.00000; // Integral Gain
      public static final double kD                 = 0.00000; // Derivative Gain
      public static final double kS                 = 0.18000; // volts 0.081073
      public static final double kV                 = 12.0 / PHYSICS.MAX_LINEAR_VELOCITY; // volts per m/s
      public static final double kA                 = 0.10000; // volts per m/s^2, free spinning
    }

    public static final class STEER_MOTOR_PROFILE {
      // FROM WPILIB SYSTEM IDENTIFICATION
      public static final double kP                 = 0.90000; // Proportion Gain
      public static final double kI                 = 0.00000; // Integral Gain
      public static final double kD                 = 0.10000; // Derivative Gain
      public static final double kS                 = 0.00000; // volts
      public static final double kV                 = 12.0 / (NEO.STATS.freeSpeedRadPerSec / STEER_MOTOR_GEARING); // volts per rad/s
      public static final double kA                 = 0.00010; // volts per rad/s^2
    }

    // TELEOPERATED
    public static final class ABSOLUTE_ROTATION_GAINS {
      public static final double kP = 4.0;
      public static final double kI = 0.0;
      public static final double kD = 0.5;
      public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0.5);
    }
    
    // MODULES
    // In order of: front left, front right, back left, back right, where the battery is in the back

    // public static final String[] MODULE_NAMES          = { "FL", "FR", "BL", "BR" };
    // public static final double[] STEER_ENCODER_OFFSETS_PROTO = { -213.047, 24.785, -34.805, -11.602 };
    // public static final double[] STEER_ENCODER_OFFSETS_COMP  = { 69.70908 + 180.0, 88.0214 + 180.0, 57.48648 + 180.0, 191.33784 + 180.0 };

    // public static final double[] STEER_ENCODER_OFFSETS_PROTO = { -213.047, 24.785, -34.805, -11.602 };
    // public static final double[] STEER_ENCODER_OFFSETS_COMP  = { 69.70908, 88.0214, 57.48648, 191.33784 };
    
    public record MODULE_CONFIG (int ID, int CAN_DRIVE, int CAN_STEER, int CAN_ENCODER, double ENCODER_OFFSET) {}

    public static final MODULE_CONFIG[] MODULES = new MODULE_CONFIG[] {
      new MODULE_CONFIG(0, 31, 32, 33, -0.061523),
      new MODULE_CONFIG(1, 34, 35, 36, 0.246582),
      new MODULE_CONFIG(2, 37, 38, 39, -0.348633),
      new MODULE_CONFIG(3, 40, 41, 42, -0.224854),
      new MODULE_CONFIG(4, 43, 44, 45, -0.340576),
      new MODULE_CONFIG(5, 46, 47, 48, -0.432373),
      new MODULE_CONFIG(6, 49, 50, 51, -0.102783),
      new MODULE_CONFIG(7, 52, 53, 54, -0.279785),
      new MODULE_CONFIG(8, 55, 56, 57, -0.002197),
      new MODULE_CONFIG(9, 58, 59, 60, -0.217041),
    };

    public static final String[] MODULE_NAMES = {
      "front-left",
      "front-right",
      "back-left",
      "back-right"
    };

    public static final MODULE_CONFIG[] EQUIPPED_MODULES_PROTOTYPE = {
      MODULES[4], // front-left
      MODULES[5], // front-right
      MODULES[6], // back-left
      MODULES[7]  // back-right
    };

    public static final MODULE_CONFIG[] EQUIPPED_MODULES_COMPETITION = {
      MODULES[0], // front-left
      MODULES[1], // front-right
      MODULES[8], // back-left
      MODULES[3]  // back-right
    };
  }

  public static final class CAN {
    // In order of: front left, front right, back left, back right, where the battery is in the back
    public static final int PDH = 1;
    public static final int SHOOTER_WHEELS_TOP = 19;
    public static final int SHOOTER_WHEELS_BOTTOM = 26;
    public static final int SHOOTER_PIVOT = 18;
    public static final int SHOOTER_FEED = 20;
    public static final int TRANSFER_OUT = 24;
    public static final int TRANSFER_IN = 22;
    public static final int INTAKE = 29; 
    public static final int AMP_PIVOT = 17;
    public static final int AMP_WHEELS = 25;
    // left/right is from the robot's view of from intake
    public static final int HANG_RIGHT = 28;

  }

  public static final class DIO {
    public static final int AMP_PIVOT = 1;
    public static final int SHOOTER_PIVOT = 2;
    public static final int BEAM_BREAK = 3;
  }

  public static final class NEO {
    public static final DCMotor STATS = new DCMotor(12.0, 3.0, 160.0, 2.065, Units.rotationsPerMinuteToRadiansPerSecond(5820), 1);

    public static final double SAFE_TEMPERATURE = 60;
    public static final int SAFE_STALL_CURRENT = 40;
    public static final int SAFE_FREE_CURRENT = 60;
    public static final double SAFE_RAMP_RATE = 0.1;

    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }
  public static final class NEO550 {
    public static final DCMotor STATS = DCMotor.getNeo550(1);
    public static final double SAFE_TEMPERATURE = 60;
    public static final int SAFE_STALL_CURRENT = 10;
    public static final double SAFE_RAMP_RATE = 0.1;
    // 
    public static double maxTorqueCurrentLimited(int currentLimit) {
      return STATS.stallTorqueNewtonMeters / STATS.stallCurrentAmps * currentLimit;
    }
  }
  public static final class SHOOTER_FEED {
    public static final double GEARING = 1.0;
    public static final double FREE_TORQUE = 1.0; // TODO
    public static final double RADIUS = Units.inchesToMeters(1.0);
  }
  public static final class SHOOTER_WHEELS {
    public static final double GEARBOX_STEP_UP = 2.0;
    public static final double ENCODER_CONVERSION_FACTOR = 2.0 * Math.PI * GEARBOX_STEP_UP;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double WHEEL_MOI = 0.00018540712;
    public static final double TOTAL_MOI = WHEEL_MOI * 12.0;
    public static final double PROJECTILE_MASS = Units.lbsToKilograms(0.5);
    public static final double COMPRESSION = Units.inchesToMeters(0.5);
    public static final double SPEED_PRECISION = Units.rotationsPerMinuteToRadiansPerSecond(10);
    public static final double TOP_EXIT_VELOCITY = 12.5;
    public static final double MAX_EXIT_VELOCITY = 13.0;
    public static final double NOTE_LOG_BASE = 1.0082;
    public static final double NOTE_LOG_OFFSET = 504.213;
    public static final double MAX_WHEEL_SPEED = NEO.STATS.freeSpeedRadPerSec * SHOOTER_WHEELS.GEARBOX_STEP_UP;

    // x is front-to-back
    // y is left-to-right
    // z is top-to-bottom
    
    public static final class PROFILE {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.0; // volts per rad/s
      public static final double kV = (12.0 / (NEO.STATS.freeSpeedRadPerSec * GEARBOX_STEP_UP)) / 0.8; // volts per rad/s
      public static final double kA = 0.0; // volts per rad/s^2
    }
  }

  public static final class SHOOTER_PIVOT {
    public static final double GEARING = 15.0 * (78.0 / 20.0) * (200.0 / 18.0);
    public static final double ROTATION_DELAY = 0.3; // seconds
    public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0.25);
    public static final Rotation2d ANGLE_PRECISION = Rotation2d.fromDegrees(0.25);
    public static final Rotation2d HEADING_PRECISION = Rotation2d.fromDegrees(0.25);
    public static final Translation3d POSITION = new Translation3d(Units.inchesToMeters(3.33), 0.0, Units.inchesToMeters(12.1));
    public static final double ABSOLUTE_POSITION_OFFSET = Units.degreesToRotations(-141.5 - 93.15 + 27); //  - [ rawAbsolutePosition from logs ] - 93.15 + [ the angle measured from the front plate of shooter ]
    public static final Rotation2d NOTE_ROTATION_OFFSET = Rotation2d.fromDegrees(-1.25); // Theoretically 3.1480961
    public static final double SHOOTER_LENGTH = Units.inchesToMeters(15.023);
    
    public static final class PROFILE {
      public static final double kP = 15.0;
      public static final double kS = 0.2;
      public static final double MAX_ACCELERATION = 30.0; // rad/s^2
    }
  }

  public static final class AMP_WHEELS {
    public static final double GEARING = 16.0 * 78.0 / 24.0;
    public static final double FREE_TORQUE = 2.5; // TODO
    public static final double RADIUS = Units.inchesToMeters(1.625 / 2.0);
  }
  public static final class AMP_PIVOT {
    public static final double GEARING = (78.0 / 10.0) * (78.0 / 16.0) * (26.0 / 12.0);
    public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(5.0);
    public static final double ABSOLUTE_POSITION_OFFSET = -0.814;

    public static final class PROFILE {
      public static final double kP = 1.5;
      public static final double kS = 0.0;
      public static final double MAX_ACCELERATION = 40.0; // rad/s^2
    }
  }

  public static final class TRANSFER {
    public static final double GEARING_IN = (42.0 / 12.0) * (24.0 / 18.0);
    public static final double GEARING_OUT = 58.0 / 12.0;
    public static final double RADIUS = Units.inchesToMeters(1.125);
    public static final double MAX_LINEAR_SPEED_IN = NEO.STATS.freeSpeedRadPerSec / GEARING_IN * RADIUS;
  }

  public static final class INTAKE {
    public static final double GEARING = (38.0 / 16.0) * (15.0 / 18.0);
    public static final double RADIUS = Units.inchesToMeters(0.5);
    public static final double MAX_LINEAR_SPEED = NEO.STATS.freeSpeedRadPerSec / GEARING * RADIUS;
  }

  public static final class HANG {
    public static final double SPOOL_RADIUS = Units.inchesToMeters((0.75 + 0.0511811024) / 2.0);
    public static final double GEARING = 16.0;
    public static final double EXTEND_HEIGHT = Units.inchesToMeters(39 - 20.5 + 11.0);
    public static final double RETRACT_HEIGHT = Units.inchesToMeters(1.0);
  }

  // LED
  public static final class LED {
    public static final int SIDE_STRIP_HEIGHT = 58; // Number of LEDs on side strip
  }
}
