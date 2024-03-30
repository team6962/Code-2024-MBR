package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Field;
import frc.robot.Constants.Preferences;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.util.software.Logging.Logger;
import frc.robot.util.software.Logging.StatusChecks;

// This class is a subsystem that controls the state of the robot. It is used to coordinate the actions of the intake, shooter, transfer, and amp subsystems.

public class RobotStateController extends SubsystemBase {
  private SwerveDrive swerveDrive;
  private Amp amp;
  private Shooter shooter;
  private Transfer transfer;
  private Intake intake;
  private DigitalInput beamBreakSensor;
  private Debouncer beamBreakDebouncer = new Debouncer(0.1);
  private Debouncer shotDebouncer = new Debouncer(0.25);
  private boolean shootOverride = false;
  // private static ShuffleboardTab tab = Shuffleboard.getTab("Auto");
  // private static SimpleWidget hasNote = tab.add("has Note", true).withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1).withPosition(0, 0);



  public enum State {
    INTAKE,
    INTAKE_AND_CENTER,
    INTAKE_OUT,
    PREPARE_AMP,
    PLACE_AMP,
    LEAVE_AMP,
    AIM_SPEAKER,
    AIM_MORTAR,
    SHOOT,
    SPIN_UP,
    PREPARE_SOURCE,
    INTAKE_SOURCE,
    PREPARE_TRAP,
    SHOOT_TRAP,
    CENTER_NOTE,
    REVERSE_SHOOTER,
    SHOOT_OVERIDE
  }

  public RobotStateController(Amp amp, SwerveDrive swerveDrive, Shooter shooter, Transfer transfer, Intake intake) {
    this.swerveDrive = swerveDrive;
    this.amp = amp;
    this.shooter = shooter;
    this.transfer = transfer;
    this.intake = intake;
    beamBreakSensor = new DigitalInput(Constants.DIO.BEAM_BREAK);

    StatusChecks.addCheck(new SubsystemBase() {}, "Beam Break Sensor", () -> beamBreakSensor.get());

    Logger.autoLog(this, "isAimed", () -> isAimed());
    Logger.autoLog(this, "hasNote", () -> hasNote());
    Logger.autoLog(this, "canShoot", () -> canShoot());
    Logger.autoLog(this, "inRange", () -> inRange());
    Logger.autoLog(this, "Loop Time", () -> Robot.getLoopTime());
    Logger.autoLog(this, "Compute Time", () -> Robot.getComputeTime());
  }

  /**
   * Sets the state of the robot
   * @param state
   * @return
   */

  public Command setState(State state) {
    switch(state) {
      case INTAKE:
        return Commands.parallel(
          intake.setState(Intake.State.IN),
          transfer.setState(Transfer.State.IN),
          amp.setState(Amp.State.DOWN)
        ).until(() -> hasNote()).raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND)).andThen(Commands.runOnce(() -> Controls.rumbleBoth().schedule()));
      case CENTER_NOTE:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN).alongWith(
            transfer.setState(Transfer.State.AMP)
          ).until(() -> !hasNote()),
          amp.setState(Amp.State.OUT).alongWith(
            transfer.setState(Transfer.State.FROM_AMP),
            intake.setState(Intake.State.SLOW_OUT)
          ).until(() -> hasNote()),
          transfer.setState(Transfer.State.FROM_AMP).alongWith(intake.setState(Intake.State.SLOW_OUT)).until(() -> !hasNote()),
          transfer.setState(Transfer.State.SLOW_IN).alongWith(intake.setState(Intake.State.SLOW_IN)).until(() -> hasNote())
        ).raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND)).andThen(Controls.rumbleBoth());
      case INTAKE_OUT:
        return transfer.setState(Transfer.State.OUT).alongWith(intake.setState(Intake.State.SLOW_OUT));
      case INTAKE_AND_CENTER:
        return setState(State.INTAKE).andThen(Commands.runOnce(() -> setState(State.CENTER_NOTE).schedule()));
      case PREPARE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN),
          amp.setState(Amp.State.IN).alongWith(
            transfer.setState(Transfer.State.AMP).alongWith(
              intake.setState(Intake.State.SLOW_IN)
            )
          ).until(() -> !hasNote()),
          Commands.parallel(
            transfer.setState(Transfer.State.AMP),
            amp.setState(Amp.State.IN)
          ).withTimeout(0.25),
          amp.setState(Amp.State.UP)
        ).raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND)).andThen(Controls.rumbleBoth());
      case PLACE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.UP),
          amp.setState(Amp.State.OUT).withTimeout(2.0)
        ).raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND)).andThen(Controls.rumbleBoth());
      case LEAVE_AMP:
        return Commands.sequence(
          amp.setState(Amp.State.DOWN)
        );
      case AIM_SPEAKER:
        return shooter.setState(Shooter.State.AIM_SPEAKER)
          .alongWith(Controls.rumbleBoth(() -> canShoot()))
          .raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND));
      case AIM_MORTAR:
        return shooter.setState(Shooter.State.AIM_MORTAR)
          .alongWith(Controls.rumbleBoth(() -> canShoot()))
          .raceWith(LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND))
          .alongWith(new ConditionalCommand(
            LEDs.setStateCommand(LEDs.State.BAD),
            Commands.runOnce(() -> {}),
            () -> !hasNote() && !RobotBase.isSimulation()
          ));
      case SHOOT:
        return Commands.sequence(
          Commands.waitUntil(() -> canShoot()),
          transfer.setState(Transfer.State.SHOOTER_FAST).alongWith(
            intake.setState(Intake.State.SLOW_IN)
          ).until(() -> !hasNote()),
          transfer.setState(Transfer.State.SHOOTER_SLOW)
        ).raceWith(
          shooter.getWheels().turnOnFeedWheels(),
          LEDs.setStateCommand(LEDs.State.RUNNING_COMMAND)
        );
      case SHOOT_OVERIDE:
        return Commands.runEnd(
          () -> shootOverride = true,
          () -> shootOverride = false
        );
      case SPIN_UP:
        return shooter.setState(Shooter.State.SPIN_UP);
      case REVERSE_SHOOTER:
        return shooter.setState(Shooter.State.REVERSE);
      default:
        return Commands.run(() -> {});
    }
  }

  public boolean hasNote() {
    // if (RobotBase.isSimulation()) {
    //   return hasNote.getEntry().getBoolean(false);
    // }
    return beamBreakDebouncer.calculate(!beamBreakSensor.get());
  }
  

  public boolean underStage() {
    // return true;
    return swerveDrive.underStage();
  }

  public Translation2d getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public boolean isAimed() {
    // if (swerveDrive.underStage()) {
    //   return 0.0;
    // }
    // if (!hasNote() && !RobotBase.isSimulation()) {
    //   return 0.0;
    // }
    
    return shooter.isAimed();
  }

  public boolean canShoot() {
    // System.out.println(isAimed());
    if (shootOverride) return true;
    return shotDebouncer.calculate(isAimed()) &&
    ((swerveDrive.getPose().getX() < ((Field.LENGTH - Field.WING_X.get()) - Constants.SWERVE_DRIVE.BUMPER_DIAGONAL / 2.0) && Constants.IS_BLUE_TEAM.get()) || (swerveDrive.getPose().getX() > (Field.LENGTH - Field.WING_X.get() + Constants.SWERVE_DRIVE.BUMPER_DIAGONAL / 2.0) && !Constants.IS_BLUE_TEAM.get()));
  }

  public boolean inRange() {
    return shooter.inRange();
  }

  @Override
  public void periodic() {
    shotDebouncer.calculate(isAimed());
    beamBreakDebouncer.calculate(!beamBreakSensor.get());

    if (RobotState.isDisabled()) {
      LEDs.setState(LEDs.State.DISABLED);
    } else {
      LEDs.setState(LEDs.State.ENABLED);
    }

    if (hasNote()) {
      LEDs.setState(LEDs.State.HAS_NOTE);
    }

    if (isAimed()) {
      LEDs.setState(LEDs.State.AIMED);
    }

    if (shooter.isAiming()) {
      if (!hasNote() && !RobotBase.isSimulation()) {
        LEDs.setState(LEDs.State.BAD);
      }
      if (shooter.inRange()) {
        LEDs.setState(LEDs.State.AIMING_IN_RANGE);
      } else {
        LEDs.setState(LEDs.State.AIMING);
      }
    }
    
    if (swerveDrive.underStage()) {
      shooter.getPivot().setMaxAngle(Preferences.SHOOTER_PIVOT.MAX_ANGLE_UNDER_STAGE);
      amp.getPivot().setMaxAngle(Preferences.AMP_PIVOT.MAX_ANGLE_UNDER_STAGE);
    } else {
      shooter.getPivot().setMaxAngle(Preferences.SHOOTER_PIVOT.MAX_ANGLE);
      amp.getPivot().setMaxAngle(Preferences.AMP_PIVOT.MAX_ANGLE);
    }
  }
}
