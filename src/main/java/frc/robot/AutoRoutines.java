package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.ShooterCommands;

public class AutoRoutines {
  private final AutoFactory m_factory;
  private final RobotContainer m_container;

  public AutoRoutines(AutoFactory factory, RobotContainer container) {
    m_factory = factory;
    m_container = container;
  }

  // returns the command group for aiming in auton
  private Command aim() {
    return ShooterCommands.AimEverythingToHub(
            m_container.turret, m_container.hood, () -> m_container.drive.getTurretPose(), 65)
        .withTimeout(0.5);
  }

  // returns the command group for shooting in auton
  private Command shoot() {
    return Commands.sequence(
        aim(),
        ShooterCommands.ShootFromDistance(
                m_container.flywheel,
                m_container.hood,
                m_container.tunnel,
                m_container.hopper,
                m_container.intake,
                () -> m_container.drive.getTurretPose(),
                65)
            .withTimeout(4),
        CommandFactory.clearJamsCommand(m_container.tunnel, m_container.hopper).withTimeout(1),
        ShooterCommands.ShootFromDistance(
                m_container.flywheel,
                m_container.hood,
                m_container.tunnel,
                m_container.hopper,
                m_container.intake,
                () -> m_container.drive.getTurretPose(),
                65)
            .withTimeout(4));
  }

  private Command deployIntake() {
    return m_container.intake.deployCommand().until(m_container.intake.intakeIsDeployed());
  }

  private Command intake() {
    return CommandFactory.intakeCommand(m_container.intake, m_container.hopper);
  }

  private Command storeIntake() {
    return m_container.intake.storeCommand().until(m_container.intake.intakeIsStored());
  }

  //   public AutoRoutine exampleRoutine() {
  //     // Creates a routine called "example" and loads a trajectory. The trajectory is essentially
  // the
  //     // path the robot will take during auton. Look at
  //     // https://choreo.autos/usage/editing-paths/#generating for an example.
  //     final AutoRoutine routine = m_factory.newRoutine("example");
  //     final AutoTrajectory exampleTraj = routine.trajectory("ExampleTraj");

  //     // If a routine was a method, you could think of this as its body.
  //     // The "routine.active())" trigger is essentially the "entrance" to a routine.
  //     // Here, you can sequence commands for the routine.
  //     routine
  //         .active()
  //         .onTrue(
  //             // Since onTrue only has one parameter, you need to use Commands.sequence to
  // schedule
  //             // more than one command.
  //             Commands.sequence(
  //                 // No semicolons! Since you are passing arguments, it is only one statement. A
  //                 // semicolon only comes at the end of a statement. Also, since you are passing
  //                 // multiple arguments, you need commas.
  //                 exampleTraj.resetOdometry(),
  //                 exampleTraj
  //                     .cmd(), // Schedule the trajectory (make the robot move on the trajectory)
  //                 // Run commands in parallel (at the same time)
  //                 m_container.drive.stopCommand(),
  //                 CommandFactory.intakeCommand(m_container.intake, m_container.hopper),
  //                 // Run flywheel then stop
  //                 ShooterCommands.ShootFromDistance(
  //                     m_container.flywheel,
  //                     m_container.tunnel,
  //                     m_container.hopper,
  //                     m_container.intake,
  //                     () -> m_container.drive.getTurretPose(),
  //                     65)));
  //     // Stop intake and indexer
  //     // m_container.intake.stopCommand(),
  //     // m_container.hopper.stopCommand();
  //     // If a routine was a method, you could think of this as its body.
  //     // The "routine.active())" trigger is essentially the "entrance" to a routine.
  //     // Here, you can sequence commands for the routine.
  //     routine
  //         .active()
  //         .onTrue(
  //             // Since onTrue only has one parameter, you need to use Commands.sequence to
  // schedule
  //             // more than one command.
  //             Commands.sequence(
  //                 // No semicolons! Since you are passing arguments, it is only one statement. A
  //                 // semicolon only comes at the end of a statement. Also, since you are passing
  //                 // multiple arguments, you need commas.
  //                 exampleTraj.resetOdometry(),
  //                 exampleTraj
  //                     .cmd(), // Schedule the trajectory (make the robot move on the trajectory)
  //                 // Run commands in parallel (at the same time)
  //                 m_container.drive.stopCommand(),
  //                 CommandFactory.intakeCommand(m_container.intake, m_container.hopper),
  //                 // Run flywheel then stop
  //                 ShooterCommands.ShootFromDistance(
  //                     m_container.flywheel,
  //                     m_container.tunnel,
  //                     m_container.hopper,
  //                     m_container.intake,
  //                     () -> m_container.drive.getTurretPose(),
  //                     65)));
  //     // Stop intake and indexer
  //     // m_container.intake.stopCommand(),
  //     // m_container.hopper.stopCommand();

  //     return routine;
  //   }
  //     return routine;
  //   }

  public AutoRoutine behindHub() {
    final AutoRoutine routine = m_factory.newRoutine("behindHub");
    final AutoTrajectory behindHubMid = routine.trajectory("BehindHubMid");
    final AutoTrajectory moveThroughDepot = routine.trajectory("MoveThroughDepotMid");
    final AutoTrajectory shootAfterDepot = routine.trajectory("ShootAfterDepotMid");

    routine.active().onTrue(
        Commands.sequence(
            shoot().withTimeout(3),
            behindHubMid.resetOdometry(),
            behindHubMid.cmd()
        )
    );

    behindHubMid.done().onTrue(moveThroughDepot.cmd());

    moveThroughDepot.active().onTrue(deployIntake());
    moveThroughDepot.active().whileTrue(intake());
    moveThroughDepot.done().onTrue(storeIntake());
    moveThroughDepot.done().onTrue(shootAfterDepot.cmd());

    // change movement to ust rotate if possible
    shootAfterDepot.done().whileTrue(shoot());

    return routine;
  }

  // Contests half of neutral zone then shoots, then does it again
  public AutoRoutine leftDblShortCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("leftDblShortCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftShortDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("LeftShortDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    // short
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(4).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(driveToMiddle.cmd());

    // short bindings will call again

    return routine;
  }

  public AutoRoutine test() {
    final AutoRoutine routine = m_factory.newRoutine("test");
    final AutoTrajectory test = routine.trajectory("test");

    routine.active().onTrue(Commands.sequence(test.resetOdometry(), test.cmd()));

    return routine;
  }

  // Contests half of neutral zone then shoots, then does it again but passes through full neutral
  // zone
  public AutoRoutine leftShortLongCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("leftShortLongCenterContest");
    final AutoTrajectory driveToMiddleShort = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddleShort =
        routine.trajectory("LeftShortDriveThroughMiddle");
    final AutoTrajectory driveBackShort = routine.trajectory("LeftShortDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    // 2 drive to middles for diff binds
    final AutoTrajectory driveToMiddleLong = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddleLong = routine.trajectory("LeftLongDriveThroughMiddle");
    final AutoTrajectory driveBackLong = routine.trajectory("LeftLongDriveBack");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3),
                driveToMiddleShort.resetOdometry(),
                driveToMiddleShort.cmd()));

    // short
    driveToMiddleShort.done().onTrue(driveThroughMiddleShort.cmd());

    driveThroughMiddleShort
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddleShort.done().onTrue(driveBackShort.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBackShort.doneFor(4).whileTrue(shoot());

    // reset
    driveBackShort.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(driveToMiddleLong.cmd());

    // long
    driveToMiddleLong.done().onTrue(driveThroughMiddleLong.cmd());

    driveThroughMiddleLong.active().onTrue(deployIntake());
    driveThroughMiddleLong.active().whileTrue(intake());
    driveThroughMiddleLong.done().onTrue(storeIntake());
    driveThroughMiddleLong.done().onTrue(driveBackLong.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBackLong.done().whileTrue(shoot());

    return routine;
  }

  // Contests half of neutral zone then shoots, then intakes from HP, then shoots
  public AutoRoutine leftShortDepotCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("leftShortDepotCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftShortDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("LeftShortDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    final AutoTrajectory moveToDepot = routine.trajectory("MoveToDepot");
    final AutoTrajectory moveThroughDepot = routine.trajectory("MoveThroughDepot");
    final AutoTrajectory shootAfterDepot = routine.trajectory("ShootAfterDepot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    // short;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(4).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(moveToDepot.cmd());

    // depot

    moveToDepot.done().onTrue(moveThroughDepot.cmd());

    moveThroughDepot.active().onTrue(deployIntake());
    moveThroughDepot.active().whileTrue(intake());
    moveThroughDepot.done().onTrue(storeIntake());
    moveThroughDepot.done().onTrue(shootAfterDepot.cmd());

    shootAfterDepot.done().whileTrue(shoot());

    return routine;
  }

  // Contests all of neutral zone then shoots
  public AutoRoutine leftDblLongCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("leftDblLongCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftLongDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("LeftLongDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));
    ;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 4 seconds after robot drives back
    driveBack.doneFor(4).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(driveToMiddle.cmd());

    // long bindings will call again

    return routine;
  }

  // Contests all of neutral zone then shoots, then then intakes from depot, then shoots
  public AutoRoutine leftLongDepotCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("leftLonbDepotCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftLongDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("LeftLongDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    final AutoTrajectory moveToDepot = routine.trajectory("MoveToDepot");
    final AutoTrajectory moveThroughDepot = routine.trajectory("MoveThroughDepot");
    final AutoTrajectory shootAfterDepot = routine.trajectory("ShootAfterDepot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                /*shoot().withTimeout(3),*/ driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    // short;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(4).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(moveToDepot.cmd());

    // depot

    moveToDepot.done().onTrue(moveThroughDepot.cmd());

    moveThroughDepot.active().onTrue(deployIntake());
    moveThroughDepot.active().whileTrue(intake());
    moveThroughDepot.done().onTrue(storeIntake());
    moveThroughDepot.done().onTrue(shootAfterDepot.cmd());

    shootAfterDepot.doneFor(4).whileTrue(shoot());

    return routine;
  }

  // Contests half of neutral zone, shoots, then climbs
  public AutoRoutine leftShortClimbCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("leftShortClimbCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftShortDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("LeftShortDriveBack");
    final AutoTrajectory climb = routine.trajectory("LeftsideClimb");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), driveToMiddle.resetOdometry(), driveToMiddle.cmd()));
    ;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! upposed to shoot 5 seconds after robot drives back then go to climb pos
    driveBack.doneFor(4).whileTrue(shoot());
    driveBack.doneDelayed(4).onTrue(climb.cmd());

    // 7s delay bc im not tryna climb too early yk
    climb.doneDelayed(7).onTrue(m_container.climber.downCommand());
    return routine;
  }

  // Contests all of neutral zone, shoots, then climbs
  public AutoRoutine leftLongClimbCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("leftLongClimbCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("LeftDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("LeftLongDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("LeftLongDriveBack");
    final AutoTrajectory climb = routine.trajectory("LeftsideClimb");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), driveToMiddle.resetOdometry(), driveToMiddle.cmd()));
    ;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! upposed to shoot 5 seconds after robot drives back then go to climb pos
    driveBack.doneFor(4).whileTrue(shoot());
    driveBack.doneDelayed(4).onTrue(climb.cmd());

    // 7s delay bc im not tryna climb too early yk
    climb.doneDelayed(7).whileTrue(m_container.climber.downCommand());
    return routine;
  }

  // Contests half of neutral zone then shoots, then does it again
  public AutoRoutine rightDblShortCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("rightDblShortCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightShortDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("RightShortDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetRight");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    // short;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(4).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(driveToMiddle.cmd());

    // short bindings will call again

    return routine;
  }

  // Contests half of neutral zone then shoots, then does it again but passes through full neutral
  // zone
  public AutoRoutine rightShortLongCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("rightShortLongCenterContest");
    final AutoTrajectory driveToMiddleShort = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddleShort =
        routine.trajectory("RightShortDriveThroughMiddle");
    final AutoTrajectory driveBackShort = routine.trajectory("RightShortDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetLeft");

    // 2 drive to middles for diff binds
    final AutoTrajectory driveToMiddleLong = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddleLong = routine.trajectory("RightLongDriveThroughMiddle");
    final AutoTrajectory driveBackLong = routine.trajectory("RightLongDriveBack");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3),
                driveToMiddleShort.resetOdometry(),
                driveToMiddleShort.cmd()));

    // short
    driveToMiddleShort.done().onTrue(driveThroughMiddleShort.cmd());

    driveThroughMiddleShort
        .active()
        .whileTrue(CommandFactory.intakeCommand(m_container.intake, m_container.hopper));
    driveThroughMiddleShort.done().onTrue(driveBackShort.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBackShort.doneFor(4).whileTrue(shoot());

    // reset
    driveBackShort.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(driveToMiddleLong.cmd());

    // long
    driveToMiddleLong.done().onTrue(driveThroughMiddleLong.cmd());

    driveThroughMiddleLong.active().onTrue(deployIntake());
    driveThroughMiddleLong.active().whileTrue(intake());
    driveThroughMiddleLong.done().onTrue(storeIntake());
    driveThroughMiddleLong.done().onTrue(driveBackLong.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBackLong.doneFor(4).whileTrue(shoot());

    return routine;
  }

  // Contests half of neutral zone then shoots, then then intakes from HP, then shoots
  public AutoRoutine rightShortHPCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("rightShortHPCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightShortDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("RightShortDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetRight");

    final AutoTrajectory moveToHP = routine.trajectory("MoveToHP");
    final AutoTrajectory shootAfterHP = routine.trajectory("ShootAfterHP");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    // short;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(4).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(moveToHP.cmd());

    // hp

    moveToHP.doneDelayed(2).onTrue(shootAfterHP.cmd());

    shootAfterHP.doneFor(4).whileTrue(shoot());

    return routine;
  }

  // Contests all of neutral zone then shoots, then does it again
  public AutoRoutine rightDblLongCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("rightDblLongCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightLongDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("RightLongDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetRight");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                /*shoot().withTimeout(3), */ driveToMiddle.resetOdometry(), driveToMiddle.cmd()));
    ;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 5 seconds after robot drives back
    driveBack.doneFor(4).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(driveToMiddle.cmd());

    // long bindings will call again

    return routine;
  }

  // Contests all of neutral zone then shoots, then intakes from HP, then shoots
  public AutoRoutine rightLongHPCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("rightShortHPCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightLongDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("RightLongDriveBack");
    final AutoTrajectory reset = routine.trajectory("ResetRight");

    final AutoTrajectory moveToHP = routine.trajectory("MoveToHP");
    final AutoTrajectory shootAfterHP = routine.trajectory("ShootAfterHP");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    // short;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! supposed to shoot 4 seconds after robot drives back
    driveBack.doneFor(4).whileTrue(shoot());

    // reset
    driveBack.doneDelayed(4).onTrue(reset.cmd());
    reset.done().onTrue(moveToHP.cmd());

    // hp

    moveToHP.doneDelayed(2).onTrue(shootAfterHP.cmd());

    shootAfterHP.done().whileTrue(shoot());

    return routine;
  }

  // Contests half of neutral zone, shoots, then climbs
  public AutoRoutine rightShortClimbCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("rightShortClimbCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightShortDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("RightShortDriveBack");
    final AutoTrajectory climb = routine.trajectory("RightsideClimb");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), driveToMiddle.resetOdometry(), driveToMiddle.cmd()));
    ;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! upposed to shoot 5 seconds after robot drives back then go to climb pos
    driveBack.doneFor(5).whileTrue(shoot());
    driveBack.doneDelayed(5).onTrue(climb.cmd());

    // 7s delay bc im not tryna climb too early yk
    climb.doneDelayed(7).whileTrue(m_container.climber.downCommand());
    return routine;
  }

  // Contests all of neutral zone, shoots, then climbs
  public AutoRoutine rightLongClimbCenterContest() {
    final AutoRoutine routine = m_factory.newRoutine("rightLongClimbCenterContest");
    final AutoTrajectory driveToMiddle = routine.trajectory("RightDriveToMiddle");
    final AutoTrajectory driveThroughMiddle = routine.trajectory("RightLongDriveThroughMiddle");
    final AutoTrajectory driveBack = routine.trajectory("RightLongDriveBack");
    final AutoTrajectory climb = routine.trajectory("RightsideClimb");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), driveToMiddle.resetOdometry(), driveToMiddle.cmd()));
    ;
    driveToMiddle.done().onTrue(driveThroughMiddle.cmd());

    driveThroughMiddle.active().onTrue(deployIntake());
    driveThroughMiddle.active().whileTrue(intake());
    driveThroughMiddle.done().onTrue(storeIntake());
    driveThroughMiddle.done().onTrue(driveBack.cmd());

    // i hope this works! upposed to shoot 5 seconds after robot drives back then go to climb pos
    driveBack.doneFor(5).whileTrue(shoot());
    driveBack.doneDelayed(5).onTrue(climb.cmd());

    // 7s delay bc im not tryna climb too early yk
    climb.doneDelayed(7).whileTrue(m_container.climber.downCommand());
    return routine;
  }

  public AutoRoutine depot() {
    final AutoRoutine routine = m_factory.newRoutine("depotLeft");
    final AutoTrajectory moveToDepot = routine.trajectory("MoveToDepot");
    final AutoTrajectory moveThroughDepot = routine.trajectory("MoveThroughDepot");
    final AutoTrajectory shootAfterDepot = routine.trajectory("ShootAfterDepot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), moveToDepot.resetOdometry(), moveToDepot.cmd()));

    // hi if ur reading this
    moveToDepot.done().onTrue(moveThroughDepot.cmd());

    moveThroughDepot.active().onTrue(deployIntake());
    moveThroughDepot.active().whileTrue(intake());
    moveThroughDepot.done().onTrue(storeIntake());
    moveThroughDepot.done().onTrue(shootAfterDepot.cmd());

    shootAfterDepot.doneFor(4).whileTrue(shoot());

    return routine;
  }

  public AutoRoutine climbDepot() {
    final AutoRoutine routine = m_factory.newRoutine("climbDepotLeft");
    final AutoTrajectory moveToDepot = routine.trajectory("MoveToDepot");
    final AutoTrajectory moveThroughDepot = routine.trajectory("MoveThroughDepot");
    final AutoTrajectory shootAfterDepot = routine.trajectory("ShootAfterDepot");
    final AutoTrajectory climb = routine.trajectory("LeftsideClimb");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                shoot().withTimeout(3), moveToDepot.resetOdometry(), moveToDepot.cmd()));

    // hi if ur reading this
    moveToDepot.done().onTrue(moveThroughDepot.cmd());

    moveThroughDepot.active().onTrue(deployIntake());
    moveThroughDepot.active().whileTrue(intake());
    moveThroughDepot.done().onTrue(storeIntake());
    moveThroughDepot.done().onTrue(shootAfterDepot.cmd());

    shootAfterDepot.done().onTrue(aim());
    shootAfterDepot.doneFor(3.5).whileTrue(shoot());
    shootAfterDepot.doneDelayed(3.5).onTrue(climb.cmd());

    // 7s delay bc im not tryna climb too early yk
    climb.doneDelayed(7).whileTrue(m_container.climber.downCommand());
    return routine;
  }

  public AutoRoutine hp() {
    final AutoRoutine routine = m_factory.newRoutine("hpRight");
    final AutoTrajectory moveToHP = routine.trajectory("MoveToHP");
    final AutoTrajectory shootAfterHP = routine.trajectory("ShootAfterHP");

    routine
        .active()
        .onTrue(
            Commands.sequence(shoot().withTimeout(3), moveToHP.resetOdometry(), moveToHP.cmd()));

    // move to shooting position 2s after it gets there. gives human player some time
    moveToHP.doneDelayed(2).onTrue(shootAfterHP.cmd());

    shootAfterHP.done().onTrue(aim());
    shootAfterHP.doneFor(3.5).whileTrue(shoot());

    return routine;
  }

  public AutoRoutine hpSimple() {
    final AutoRoutine routine = m_factory.newRoutine("hpRight");
    final AutoTrajectory moveToHP = routine.trajectory("MoveToHP");

    routine.active().onTrue(Commands.sequence(moveToHP.resetOdometry(), moveToHP.cmd()));

    // move to shooting position 2s after it gets there. gives human player some time
    moveToHP.done().onTrue(shoot());

    return routine;
  }

  public AutoRoutine climbhp() {
    final AutoRoutine routine = m_factory.newRoutine("climbhpRight");
    final AutoTrajectory moveToHP = routine.trajectory("MoveToHP");
    final AutoTrajectory shootAfterHP = routine.trajectory("ShootAfterHP");
    final AutoTrajectory climb = routine.trajectory("RightsideClimb");

    routine
        .active()
        .onTrue(
            Commands.sequence(shoot().withTimeout(3), moveToHP.resetOdometry(), moveToHP.cmd()));

    // move to shooting position 2s after it gets there. gives human player some time
    moveToHP.doneDelayed(2).onTrue(shootAfterHP.cmd());

    shootAfterHP.doneFor(5).whileTrue(shoot());
    shootAfterHP.doneDelayed(5).onTrue(climb.cmd());

    // 7s delay bc im not tryna climb too early yk
    climb.doneDelayed(7).whileTrue(m_container.climber.downCommand());
    return routine;
  }

  public AutoRoutine standstill() {
    final AutoRoutine routine = m_factory.newRoutine("standstill");
    final AutoTrajectory moveToHP = routine.trajectory("MoveToHP");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.deadline(
                    new WaitCommand(15),
                    Commands.sequence(
                        Commands.parallel(
                            ShooterCommands.AimEverythingToHub(
                                m_container.turret,
                                m_container.hood,
                                () -> m_container.drive.getPose(),
                                65),
                            Commands.sequence(
                                new WaitCommand(1),
                                ShooterCommands.ShootFromDistance(
                                    m_container.flywheel,
                                    m_container.hood,
                                    m_container.tunnel,
                                    m_container.hopper,
                                    m_container.intake,
                                    () -> m_container.drive.getTurretPose(),
                                    65))))),
                moveToHP.resetOdometry(),
                moveToHP.cmd()));

    return routine;
  }

  public AutoRoutine standstillunjam() {
    final AutoRoutine routine = m_factory.newRoutine("standstill");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                    ShooterCommands.AimEverythingToHub(
                        m_container.turret,
                        m_container.hood,
                        () -> m_container.drive.getPose(),
                        65),
                    Commands.sequence(
                        new WaitCommand(1),
                        ShooterCommands.ShootFromDistance(
                                m_container.flywheel,
                                m_container.hood,
                                m_container.tunnel,
                                m_container.hopper,
                                m_container.intake,
                                () -> m_container.drive.getTurretPose(),
                                65)
                            .withTimeout(4),
                        CommandFactory.clearJamsCommand(m_container.tunnel, m_container.hopper)
                            .withTimeout(1),
                        ShooterCommands.ShootFromDistance(
                                m_container.flywheel,
                                m_container.hood,
                                m_container.tunnel,
                                m_container.hopper,
                                m_container.intake,
                                () -> m_container.drive.getTurretPose(),
                                65)
                            .withTimeout(4),
                        CommandFactory.clearJamsCommand(m_container.tunnel, m_container.hopper)
                            .withTimeout(1),
                        ShooterCommands.ShootFromDistance(
                                m_container.flywheel,
                                m_container.hood,
                                m_container.tunnel,
                                m_container.hopper,
                                m_container.intake,
                                () -> m_container.drive.getTurretPose(),
                                65)
                            .withTimeout(4),
                        CommandFactory.clearJamsCommand(m_container.tunnel, m_container.hopper)
                            .withTimeout(1)))));

    return routine;
  }

  public AutoRoutine test2() {
    AutoRoutine routine = m_factory.newRoutine("test2");

    AutoTrajectory traj = routine.trajectory("test2");

    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }

  public AutoRoutine moveTwoMeters() {
    AutoRoutine routine = m_factory.newRoutine("moveTwoMeters");

    AutoTrajectory traj = routine.trajectory("straight1meter");

    routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

    return routine;
  }
}
