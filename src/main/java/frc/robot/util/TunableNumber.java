package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TunableNumber extends SubsystemBase {
  private String name;
  private Consumer<Double> setter;
  private double currentNumber;
  private GenericEntry entry;
  private ShuffleboardTab tab;

  public TunableNumber(SubsystemBase subsystem, String name, Consumer<Double> setter, double defaultNumber) {
    this.name = subsystem.getClass().getSimpleName() + "/" + name;
    this.setter = setter;
    tab = Shuffleboard.getTab("Tunable Numbers");
    currentNumber = defaultNumber;
    entry = tab.add(name, currentNumber)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .getEntry();
  }

  @Override
  public void periodic() {
    double new_n = entry.getDouble(0);

    if (new_n != currentNumber) {
      setter.accept(new_n);
      currentNumber = new_n;
    }
  }
}