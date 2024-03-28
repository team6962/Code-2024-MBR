// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Preferences {
  public static final class SWERVE_DRIVE {
    public static final double   TELEOPERATED_FINE_TUNE_DRIVE_POWER = 0.1; // Percent driving power when using d-pad
    public static final double   TELEOPERATED_DRIVE_POWER           = 0.5; // Percent driving power
    public static final double   TELEOPERATED_BOOST_POWER           = 1.0; // Percent power when using the triggers
    public static final double   TELEOPERATED_ROTATE_POWER          = 0.5; // Percent rotating power
    public static final double   TELEOPERATED_SHOOTER_SPEED         = 1.5; // m/s
  }

  public final class NOTE_DETECTION {
    public static final double THRESHOLD = 1.0;
  }

  public static final class SHOOTER_FEED {
    public static final double POWER_SHOOT = 1.0;
    public static final double POWER_IN = 0.5;
  }
  
  public static final class SHOOTER_WHEELS {
    public static final double TARGET_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(10000);
  }

  public static final class SHOOTER_PIVOT {
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(60.0); // 63.0
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(27.5); // 21.148 (really 21.6626)
    public static final Rotation2d IDLE_ANGLE = Rotation2d.fromDegrees(30.0);
    public static final Rotation2d MAX_ANGLE_UNDER_STAGE = Rotation2d.fromDegrees(50.0);
  }

  public static final class AMP_WHEELS {
    public static final double POWER = 1.0;
  }

  public static final class AMP_PIVOT {
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(90.0);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-60.0);
    public static final Rotation2d OUTPUT_ANGLE = Rotation2d.fromDegrees(68.0);
    public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromDegrees(-50.0);
    public static final Rotation2d MAX_ANGLE_UNDER_STAGE = Rotation2d.fromDegrees(-20.0);
  }

  public static final class TRANSFER {
    public static final double IN_POWER = 0.5;
    public static final double SLOW_IN_POWER = 0.5;
    public static final double OUT_POWER_BOTTOM = 0.5;
    public static final double OUT_POWER_TOP = 1.0;
    public static final double TO_SHOOTER_FAST_POWER = 0.2;
    public static final double TO_SHOOTER_SLOW_POWER = 0.1;
    public static final double TO_AMP_POWER = 0.4;
  }

  public static final class INTAKE {
    public static final double IN_POWER = 0.5;
    public static final double TO_SHOOTER_POWER = 0.3;
    public static final double SLOW_OUT_POWER = 0.4;
  }

  public static final class HANG {
    public static final double LEFT_MOTOR_EXTEND_POWER = 1.0; 
    public static final double RIGHT_MOTOR_EXTEND_POWER = 1.0; 

    public static final double LEFT_MOTOR_RETRACT_POWER = -1.0;
    public static final double RIGHT_MOTOR_RETRACT_POWER = -1.0;    
  }

  public static final class VOLTAGE_LADDER {
    public static final double SWERVE_DRIVE = 7.0;
    public static final double SHOOTER = 8.0;
    public static final double INTAKE = 8.5;
    public static final double TRANSFER = 9.0;
    public static final double HANG = 9.0;
    public static final double AMP = 10.0;
  }
}