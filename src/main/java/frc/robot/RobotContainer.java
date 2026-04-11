// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  final DoubleEntry targetHoodAngleEntry;
  final DoubleEntry softwareHoodAngleEntry;
  final DoubleEntry efficiencyFactorEntry;
  // Subsystems
  public final Vision vision;
  public final Drive drive;
  public final Climber climber =
      new Climber(
          Constants.ClimberConstants.CLIMBER_MOTOR_ID_LEFT,
          Constants.ClimberConstants.CLIMBER_MOTOR_ID_RIGHT);
  public final Turret turret =
      new Turret(Constants.TurretConstants.TURRET_MOTOR_ID, Constants.currentMode);
  // new Turret(Constants.TurretConstants.TURRET_MOTOR_ID,
  // Constants.TurretConstants.TURRET_CANCODER_ID, Constants.currentMode);

  public final Hood hood = new Hood(Constants.HoodConstants.HOOD_MOTOR_ID, Constants.currentMode);
  public final Flywheel flywheel =
      new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Constants.currentMode);
  public final Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
  public final Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
  public final Tunnel tunnel =
      new Tunnel(
          Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
          Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
  public final Telemetry telemetry;

  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final VorTXControllerXbox driverController = new VorTXControllerXbox(0);
  private final VorTXControllerXbox operatorController = new VorTXControllerXbox(1);
  private final VorTXControllerXbox sysIdController = new VorTXControllerXbox(2);

  // Auton
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  final AutoChooser autonChooser = new AutoChooser();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> sysIdChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // SignalLogger.start();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {});

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    VisionConstants.frontCameraName, VisionConstants.frontCameraTransform),
                new VisionIOPhotonVision(
                    VisionConstants.backCameraName, VisionConstants.backCameraTransform),
                new VisionIOPhotonVision(
                    VisionConstants.leftCameraName, VisionConstants.leftCameraTransform),
                new VisionIOPhotonVision(
                    VisionConstants.rightCameraName, VisionConstants.rightCameraTransform));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.frontCameraName,
                    VisionConstants.frontCameraTransform,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.backCameraName,
                    VisionConstants.backCameraTransform,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.leftCameraName,
                    VisionConstants.leftCameraTransform,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.rightCameraName,
                    VisionConstants.rightCameraTransform,
                    driveSimulation::getSimulatedDriveTrainPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});

        vision = new Vision(drive, new VisionIO() {});
    }
    telemetry =
        new Telemetry(drive, vision, flywheel, hood, turret, hopper, intake, tunnel, climber);
    // Init auton objects
    autoFactory = drive.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory, this);

    // Set up auto routines
    autonChooser.addRoutine("Standstill Shoot Unjam", autoRoutines::standstillunjam);
    autonChooser.addRoutine("Standstill shoot then move", autoRoutines::standstill);
    // autonChooser.addRoutine("test2", autoRoutines::test2);

    // autonChooser.addRoutine(
    //     "Left DblShort Center Contest", autoRoutines::leftDblShortCenterContest);
    // autonChooser.addRoutine(
    //     "Left ShortLong Center Contest", autoRoutines::leftShortLongCenterContest);
    // autonChooser.addRoutine(
    //     "Left ShortDepot Center Contest", autoRoutines::leftShortDepotCenterContest);
    // autonChooser.addRoutine("Left DblLong Center Contest",
    // autoRoutines::leftDblLongCenterContest);
    // autonChooser.addRoutine(
    //     "Left LongDepot Center Contest", autoRoutines::leftLongDepotCenterContest);

    // autonChooser.addRoutine(
    //     "Right DblShort Center Contest", autoRoutines::rightDblShortCenterContest);
    // autonChooser.addRoutine(
    //     "Right ShortLong Center Contest", autoRoutines::rightShortLongCenterContest);
    // autonChooser.addRoutine(
    //     "Right ShortHP Center Contest", autoRoutines::rightShortHPCenterContest);
    // autonChooser.addRoutine(
    //     "Right DblLong Center Contest", autoRoutines::rightDblLongCenterContest);
    // autonChooser.addRoutine("Right LongHP Center Contest",
    // autoRoutines::rightLongHPCenterContest);

    // autonChooser.addRoutine(
    //     "Left Short Climb Center Contest", autoRoutines::leftShortClimbCenterContest);
    // autonChooser.addRoutine(
    //     "Left Long Climb Center Contest", autoRoutines::leftLongClimbCenterContest);
    // autonChooser.addRoutine(
    //     "Right Short Climb Center Contest", autoRoutines::rightShortClimbCenterContest);
    // autonChooser.addRoutine(
    //     "Right Long Climb Center Contest", autoRoutines::rightLongClimbCenterContest);

    // autonChooser.addRoutine("Depot (Left)", autoRoutines::depot);
    // autonChooser.addRoutine("Human Player Intake (Right)", autoRoutines::hp);

    // autonChooser.addRoutine("Climb Depot (Left)", autoRoutines::climbDepot);
    // autonChooser.addRoutine("Climb Human Player Intake (Right)", autoRoutines::climbhp);

    autonChooser.addRoutine("HP Simple", autoRoutines::hpSimple);
    // autonChooser.addRoutine("Move two meters", autoRoutines::moveTwoMeters);

    SmartDashboard.putData("Auton Chooser", autonChooser);

    RobotModeTriggers.autonomous().whileTrue(autonChooser.selectedCommandScheduler());
    // Init SysId object
    sysIdChooser = new LoggedDashboardChooser<>("SysId Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    sysIdChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    sysIdChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    sysIdChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    sysIdChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    sysIdChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    sysIdChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // Create the SysId routine
    var sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (s) -> Logger.recordOutput("Turret/SysIdTestState", s.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> turret.setVoltage(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                turret));
    sysIdChooser.addOption(
        "turret SysId (Quasistatic Forward)",
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
    sysIdChooser.addOption(
        "turret SysId (Quasistatic Reverse)",
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
    sysIdChooser.addOption(
        "turret SysId (Dynamic Forward)", sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
    sysIdChooser.addOption(
        "turret SysId (Dynamic Reverse)", sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable shooterTable = inst.getTable("Subystems/Shooter");
    targetHoodAngleEntry = shooterTable.getDoubleTopic("targetHoodAngleEntry").getEntry(65);
    targetHoodAngleEntry.set(65);
    softwareHoodAngleEntry = shooterTable.getDoubleTopic("softwareHoodAngleEntry").getEntry(65);
    softwareHoodAngleEntry.set(65);
    efficiencyFactorEntry = shooterTable.getDoubleTopic("efficiencyFactorEntry").getEntry(1.03);
    efficiencyFactorEntry.set(1.03);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // do not move the defaultcommands
    intake.setDefaultCommand(intake.stopCommand().withName("stop intake"));
    climber.setDefaultCommand(climber.stopCommand().withName("stop climber"));
    hopper.setDefaultCommand(hopper.stopCommand().withName("stop hopper"));
    hood.setDefaultCommand(hood.stopCommand().withName("stop hood"));
    flywheel.setDefaultCommand(flywheel.stopCommand().withName("stop flywheel"));
    tunnel.setDefaultCommand(tunnel.stopCommand().withName("stop tunnel"));
    // turret.setDefaultCommand(ShooterCommands.AimToHubOrSide(turret, () ->
    // drive.getTurretPose()));
    // turret.setDefaultCommand(ShooterCommands.AimToHub(turret, () -> drive.getTurretPose()));
    // turret.setDefaultCommand(turret.stopCommand().withName("stop turret"));
    turret.setDefaultCommand(
        ShooterCommands.SOTMAim(drive.getPose(), drive.getFieldRelativeVelocity(), turret, hood));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
                drive,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX())
            .withName("joystick drive"));

    // Reset gyro / odometry
    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
    driverController.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));
    // Set bindings

    // Shooter Binds
    // driverController.lb.whileTrue(turret.moveCommand(true));
    // driverController.rb.whileTrue(turret.moveCommand(false));

    // driverController.rt.whileTrue(
    //     ShooterCommands.ShootFromDistance(
    //         flywheel,
    //         hood,
    //         tunnel,
    //         hopper,
    //         intake,
    //         () -> drive.getTurretPose().transformBy(new Transform2d(drive.getFieldRelativeVelocity(), new Rotation2d())),
    //         65));

    driverController.rt.whileTrue(
        ShooterCommands.SOTMShoot(
            drive.getPose(), drive.getFieldRelativeVelocity(), tunnel, hopper, flywheel, intake));

    // driverController.lt.whileTrue(
    // ShooterCommands.ShootFromDistanceBackwardsHopper(
    //     flywheel, tunnel, hopper, intake, () -> drive.getTurretPose(), 65));
    // controller.rt.whileTrue(
    //     CommandFactory.shootCommand(
    //         flywheel, tunnel, hopper, () -> flywheel.flywheelSpeedEntry.getAsDouble() * 90));
    driverController.povLeft.whileTrue(hood.moveCommand(true));
    driverController.povRight.whileTrue(hood.moveCommand(false));
    driverController.lt.whileTrue(
        ShooterCommands.AimEverythingToHub(
            turret, hood, () -> drive.getTurretPose(), targetHoodAngleEntry.getAsDouble()));
    // driverController
    //     .yButton
    //     .toggleOnTrue(
    //         Commands.parallel(
    //             ShooterCommands.AimToSide(turret, () -> drive.getPose()),
    //             hood.setPositionPIDCommand(targetHoodAngleEntry.getAsDouble() - 5)))
    //     .onFalse(hood.setPositionPIDCommand(targetHoodAngleEntry.getAsDouble()));
    // Operator Shooter Binds
    operatorController.bButton.onTrue(new InstantCommand(() -> ShooterCommands.offset += 0.01));
    operatorController.xButton.onTrue(new InstantCommand(() -> ShooterCommands.offset -= 0.01));
    operatorController.aButton.toggleOnTrue(
        ShooterCommands.AimToHub(turret, () -> drive.getTurretPose()).withName("aim hub"));
    operatorController.yButton.toggleOnTrue(
        ShooterCommands.AimToSide(turret, () -> drive.getPose()).withName("aim side"));
    operatorController.lb.whileTrue(turret.moveCommand(true));
    operatorController.rb.whileTrue(turret.moveCommand(false));
    operatorController.povDown.whileTrue(hood.moveCommand(false));
    operatorController.povUp.whileTrue(hood.moveCommand(true));
    // operatorController.lt.whileTrue(
    //     CommandFactory.manualShootCommandAtSpeed(flywheel, hopper, tunnel, () -> 0.1));
    // operatorController.rt.whileTrue(
    //     ShooterCommands.AimEverythingToHub(
    //         turret,
    //         hood,
    //         () -> ShooterCommands.getTurretPose(() -> drive.getPose()).toPose2d(),
    //         targetHoodAngleEntry.getAsDouble()));
    // Climber Binds
    driverController.povUp.whileTrue(climber.upCommand());
    driverController.povDown.whileTrue(climber.downCommand());

    // Intake Binds
    driverController.xButton.whileTrue(CommandFactory.intakeCommand(intake, hopper));
    driverController.bButton.whileTrue(intake.outtakeCommand());
    driverController.aButton.whileTrue(CommandFactory.clearJamsCommand(tunnel, hopper));

    // Test/Misc Binds
    // driverController.rs.onTrue(new RunCommand(() -> hood.zeroHood(), hood));
    operatorController.view.onTrue(new InstantCommand(() -> turret.zero()));
    driverController.menu.onTrue(new InstantCommand(() -> drive.zeroDriveTrain()));

    sysIdController.lb.onTrue(Commands.runOnce(SignalLogger::start));
    sysIdController.rb.onTrue(Commands.runOnce(SignalLogger::stop));
    // sysIdController.povUp.whileTrue();
    sysIdController.yButton.whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    sysIdController.aButton.whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    sysIdController.xButton.whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    sysIdController.bButton.whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.selectedCommand();
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Hub",
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
            ? Constants.FieldConstants.RED_HUB_POSE3D
            : Constants.FieldConstants.BLUE_HUB_POSE3D);
    Logger.recordOutput(
        "Turret/simulatedPose", ShooterCommands.getTurretPose(() -> drive.getPose()));

    Logger.recordOutput(
        "Shooter/flywheelPose",
        ShooterCommands.getFlywheelPose(
            () -> drive.getPose(), () -> turret.getTurretCurrentPosition()));
  }
}
