package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;

public class CommandFactory {

  public static Command manualShootCommand(Flywheel flywheel, Tunnel tunnel) {
    return Commands.parallel(
            flywheel.shootCommand(),
            Commands.sequence(
                new WaitUntilCommand(() -> flywheel.isAtSpeed()), tunnel.intakeCommand()))
        .withName("manual shoot command group");
  }

  public static Command shootCommand(Flywheel flywheel, Tunnel tunnel, double targetRPS) {
    return Commands.parallel(
            flywheel.shootCommand(targetRPS),
            Commands.sequence(
                new WaitUntilCommand(() -> flywheel.isAtSpeed()), tunnel.intakeCommand()))
        .withName("shoot command group");
  }

  public static Command intakeCommand(Intake intake, Hopper hopper) {
    return Commands.parallel(intake.intakeCommand(), hopper.intakeCommand())
        .withName("intake command group");
  }

  public static Command outtakeCommand(Intake intake, Hopper hopper) {
    return Commands.parallel(intake.outtakeCommand(), hopper.outtakeCommand())
        .withName("outtake command group");
  }

  public static Command clearJamsCommand(Tunnel tunnel, Hopper hopper) {
    return Commands.parallel(tunnel.outtakeCommand(), hopper.outtakeCommand())
        .withName("clear jams");
  }
}
