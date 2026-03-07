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
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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
  private final VorTXControllerXbox controller = new VorTXControllerXbox(0);

  // Auton
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  final AutoChooser autonChooser = new AutoChooser();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> sysIdChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    autonChooser.addRoutine("Center Contest Left", autoRoutines::centerContestLeft);
    autonChooser.addRoutine("Center Contest Right", autoRoutines::centerContestRight);
    autonChooser.addRoutine("Depot (Left)", autoRoutines::depotLeft);
    autonChooser.addRoutine("Human Player (Right)", autoRoutines::hpRight);

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
    // hood.setDefaultCommand(hood.hold().withName("hold hood"));
    // flywheel.setDefaultCommand(flywheel.stopCommand().withName("hold flywheel
    // velocity"));
    // turret.setDefaultCommand(
    // TurretCommands.AimToHub(turret, () -> drive.getPose()).withName("aim to
    // hub"));
    hood.setDefaultCommand(hood.stopCommand().withName("stop hood"));
    flywheel.setDefaultCommand(flywheel.stopCommand().withName("stop flywheel"));
    tunnel.setDefaultCommand(tunnel.stopCommand().withName("stop tunnel"));
    turret.setDefaultCommand(turret.stopCommand().withName("stop turret"));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX())
            .withName("joystick drive"));

    // Reset gyro / odometry
    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
    controller.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

    // Set bindings
    controller.lb.whileTrue(turret.moveCommand(true));
    controller.rb.whileTrue(turret.moveCommand(false));
    controller.rt.whileTrue(CommandFactory.manualShootCommand(flywheel, hopper, tunnel));
    // controller.rt.whileTrue(flywheel.shootCommand());
    controller.povLeft.whileTrue(hood.moveCommand(true));
    controller.povRight.whileTrue(hood.moveCommand(false));
    // controller.yButton.whileTrue(ShooterCommands.AimToHub(turret, () -> drive.getPose()));
    // controller.aButton.whileTrue(
    // ShooterCommands.AimToHub(turret, flywheel, hood, () -> drive.getPose(), 65));
    // controller.yButton.whileTrue(hood.moveCommand(true));
    // controller.aButton.whileTrue(hood.moveCommand(false)

    // Climber Binds
    controller.povUp.whileTrue(climber.upCommand());
    controller.povDown.whileTrue(climber.downCommand());

    // Intake Binds
    controller.xButton.whileTrue(CommandFactory.intakeCommand(intake, hopper));
    controller.bButton.whileTrue(CommandFactory.outtakeCommand(intake, hopper));

    controller.aButton.whileTrue(CommandFactory.clearJamsCommand(tunnel, hopper));

    controller.povUp.whileTrue(turret.setPositionPIDCommand(0.1));

    controller.menu.onTrue(new InstantCommand(() -> turret.zero()));
    // controller.menu.onTrue(drive.runOnce(() -> drive.))
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
        "Turret/simulatedPose",
        new Pose3d(
                driveSimulation
                    .getSimulatedDriveTrainPose()
                    .plus(
                        new Transform2d(
                            0.13, -0.2, new Rotation2d(turret.getTurretCurrentPosition() * 2 * Math.PI))))
            .plus(new Transform3d(0, 0, 0.3, new Rotation3d())));
    Logger.recordOutput(
        "Hood/simulatedPose",
        new Pose3d(
                driveSimulation
                    .getSimulatedDriveTrainPose()
                    .plus(
                        new Transform2d(
                            0.13, -0.2, new Rotation2d(turret.getTurretCurrentPosition() * 2 * Math.PI))))
            .plus(
                new Transform3d(0, 0, 0.3, new Rotation3d(0, hood.getHoodAngle() * Math.PI / 180, 0))));
    Logger.recordOutput(
        "Turret/targetPose",
        new Pose3d(
                driveSimulation
                    .getSimulatedDriveTrainPose()
                    .plus(
                        new Transform2d(
                            0.13, -0.2, new Rotation2d(turret.getTurretTargetPosition() * 2 * Math.PI))))
            .plus(new Transform3d(0, 0, 0.3, new Rotation3d())));
  }
}
