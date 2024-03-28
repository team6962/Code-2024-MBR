package frc.robot.util.software.Logging;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatusChecks {
  public static int row = 0;
  public static int column = 1;
  private static ShuffleboardTab tab = Shuffleboard.getTab("Status Checks");
  private static ComplexWidget refreshButton = tab.add("Refresh", StatusChecks.refresh()).withWidget(BuiltInWidgets.kCommand).withSize(1, 1).withPosition(0, 0);
  private static Map<String, BooleanSupplier> suppliers = new HashMap<String, BooleanSupplier>();
  private static Map<String, GenericEntry> entries = new HashMap<String, GenericEntry>();

  private static void addCheck(String name, BooleanSupplier supplier) {
    suppliers.put(name, supplier);
    entries.put(name, tab.add(name.replace("/", " "), supplier.getAsBoolean()).withWidget(BuiltInWidgets.kBooleanBox).withSize(1, 1).withPosition(column, row).getEntry());
    column++;
    if (column > 12) {
      column = 0;
      row++;
    }
  }

  public static void addCheck(SubsystemBase subsystem, String name, BooleanSupplier supplier) {
    addCheck(subsystem.getClass().getSimpleName() + "/" + name, supplier);
    
  }

  public static Command refresh() {
    return Commands.runOnce(() -> {
      for (Map.Entry<String, BooleanSupplier> supplier : suppliers.entrySet()) {
        entries.get(supplier.getKey()).setBoolean(supplier.getValue().getAsBoolean());
      }
    }).ignoringDisable(true);
  }
}