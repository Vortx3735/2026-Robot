package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoRoutines {
  private final AutoFactory m_factory;

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine exampleRoutine() {
    // Creates a routine called "example" and loads a trajectory. The trajectory is essentially the
    // path the robot will take during auton. Look at
    // https://choreo.autos/usage/editing-paths/#generating for an example.
    final AutoRoutine routine = m_factory.newRoutine("example");
    final AutoTrajectory exampleTraj = routine.trajectory("ExampleTraj");

    // If a routine was a method, you could think of this as its body.
    // The "routine.active())" trigger is essentially the "entrance" to a routine.
    // Here, you can sequence commands for the routine.
    routine
        .active()
        .onTrue(
            // Since onTrue only has one parameter, you need to use Commands.sequence to schedule
            // more than one command.
            Commands.sequence(
                // No semicolons! Since you are passing arguments, it is only one statement. A
                // semicolon only comes at the end of a statement. Also, since you are passing
                // multiple arguments, you need commas.
                exampleTraj.resetOdometry(),
                exampleTraj.cmd(), // Schedule the trajectory (make the robot move on the trajectory)
                // Run commands in parallel (at the same time)
                Commands.parallel(
                    // Run intake and indexer
                    RobotContainer.intake.intakeCommand(),
                    RobotContainer.indexer.runCommand(1)),
                // Stop intake and indexer
                RobotContainer.intake.stopCommand(),
                RobotContainer.indexer.stopCommand(),
                // Run flywheel then stop
                RobotContainer.flywheel.shootCommand(),
                RobotContainer.flywheel.stopCommand()));

    return routine;
  }
}