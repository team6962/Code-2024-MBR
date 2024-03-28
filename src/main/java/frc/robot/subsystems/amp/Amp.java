// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.amp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ENABLED_SYSTEMS;
import frc.robot.Constants.Preferences;

public class Amp extends SubsystemBase {
  private AmpPivot pivot;
  private AmpWheels wheels;
 
  public static enum State {
    OUT,
    IN,
    DOWN,
    UP,
  }

  public Amp() {
    pivot = new AmpPivot();
    wheels = new AmpWheels();
  }

  @Override
  public void periodic() {
    if (!ENABLED_SYSTEMS.ENABLE_AMP) return;
  }

  public AmpPivot getPivot() {
    return pivot;
  }

  public AmpWheels getWheels() {
    return wheels;
  }

  public Command setState(State state) {
    switch(state) {
      case IN:
        return wheels.setState(AmpWheels.State.IN);
      case DOWN:
        return pivot.setTargetAngleCommand(Preferences.AMP_PIVOT.INTAKE_ANGLE).until(() -> pivot.doneMoving());
      case UP:
        return pivot.setTargetAngleCommand(Preferences.AMP_PIVOT.OUTPUT_ANGLE).until(() -> pivot.doneMoving());
      case OUT:
        return wheels.setState(AmpWheels.State.OUT);
    }
    return null;
  }

  public boolean doneMoving() {
    return pivot.doneMoving();
  }
}
