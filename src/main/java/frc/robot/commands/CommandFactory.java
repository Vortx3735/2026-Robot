package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandFactory {
  public static Command exampleCommand() {
    return Commands.run(() -> {});
  }
}
