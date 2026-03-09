package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import java.util.function.Supplier;

public class CommandFactory {
  public static Command runTunnelAndHopperCommand(Tunnel tunnel, Hopper hopper) {
    return Commands.parallel(tunnel.intakeCommand(), hopper.intakeCommand());
  }

  public static Command manualShootCommand(Flywheel flywheel, Hopper hopper, Tunnel tunnel) {
    return Commands.parallel(
            flywheel.shootCommand(),
            Commands.sequence(
                new WaitUntilCommand(flywheel.isAtSpeed()),
                Commands.parallel(hopper.intakeCommand(), tunnel.intakeCommand())))
        .withName("manual shoot command group");
  }

  public static Command shootCommand(
      Flywheel flywheel, Tunnel tunnel, Hopper hopper, Supplier<Double> targetRPS) {
    return Commands.parallel(
            flywheel.shootCommand(targetRPS),
            hopper.intakeCommand(),
            // Commands.either(tunnel.intakeCommand(), tunnel.stopCommand(), flywheel.isAtSpeed()))
            Commands.sequence(new WaitUntilCommand(flywheel.isAtSpeed()), tunnel.intakeCommand()))
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
