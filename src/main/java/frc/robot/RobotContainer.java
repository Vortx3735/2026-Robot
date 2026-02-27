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

  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final VorTXControllerXbox controller = new VorTXControllerXbox(0);

  // Auton
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  final AutoChooser autonChooser = new AutoChooser();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
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

        VisionIO visionIO;
        try {
          visionIO =
              new VisionIOPhotonVisionSim(
                  camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose);
        } catch (UnsatisfiedLinkError e) {
          // If the JNI library (like PhotonVision) fails to load (e.g. in CI
          // environment),
          // fallback to a dummy implementation to allow the robot code to start up.
          visionIO = new VisionIO() {};
          System.err.println(
              "Warning: Failed to load VisionIOPhotonVisionSim, falling back to dummy implementation. Error: "
                  + e.getMessage());
        }
        vision = new Vision(drive, visionIO);
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
    // Init auton objects
    autoFactory = drive.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory, this);
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autonChooser.addRoutine(
        "Example Auton",
        autoRoutines
            ::exampleRoutine); // Logged dashboard chooser no support Choreo AutoRoutine objects
    SmartDashboard.putData("Auton Chooser", autonChooser);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
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
    autoChooser.addOption(
        "turret SysId (Quasistatic Forward)",
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "turret SysId (Quasistatic Reverse)",
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "turret SysId (Dynamic Forward)", sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
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

    /*
     * // Test Binds
     *
     * // controller.povUp.whileTrue(flywheel.setVelocityPIDCommand(100));
     * // controller.povDown.whileTrue(flywheel.setVelocityPIDCommand(0));
     * // controller.povRight.whileTrue(turret.moveCommand(0.5));
     * // controller.povLeft.whileTrue(turret.moveCommand(-0.5));
     * // controller.yButton.whileTrue(hood.setPositionPIDCommand(-45));
     * // controller.bButton.whileTrue(hood.setPositionPIDCommand(0));
     * // controller.rt.whileTrue(hopper.runHopperCommand(true));
     * controller.lt.whileTrue(intake.intakeCommand());
     * // controller.aButton.whileTrue(hopper.runHopperCommand(true));
     * // controller.xButton.whileTrue(TurretCommands.AimToSide(turret, () ->
     * drive.getPose()));
     *
     * controller.povUp.whileTrue(climber.upCommand());
     * // controller.povRight.whileTrue(hopper.runHopperCommand(true));
     * controller.povLeft.whileTrue(turret.setPositionPIDCommandManualSetpoint());
     * // controller.povRight.whileTrue(TurretCommands.AimToSide(turret, () ->
     * drive.getPose()));
     * controller.povDown.whileTrue(climber.downCommand());
     * controller.lb.whileTrue(hopper.runHopperCommand(true));
     * // controller.rt.whileTrue(flywheel.setVelocityPIDCommand());
     * controller.rt.whileTrue(CommandFactory.shootCommand(flywheel, tunnel));
     * controller.rb.whileTrue(hopper.runHopperCommand(false));
     *
     * controller.yButton.whileTrue(hood.moveCommand(true));
     * controller.xButton.whileTrue(turret.moveCommand(true));
     * controller.bButton.whileTrue(turret.moveCommand(false));
     * controller.aButton.whileTrue(hood.moveCommand(false));
     *
     * controller.menu.onTrue(new InstantCommand(() -> turret.zero()));
     */

    /*
     * // Actual Binds
     */
    // Shooter Binds
    // controller.lt.onTrue(TurretCommands.AimToSide(turret, () ->
    // drive.getPose())); does AimToSide
    // aim to a side of the field?
    controller.lb.whileTrue(turret.moveCommand(true));
    // controller.rb.whileTrue(turret.moveCommand(false));
    controller.rb.whileTrue(CommandFactory.manualShootCommand(flywheel, tunnel));
    controller.povLeft.whileTrue(hood.moveCommand(true));
    controller.povRight.whileTrue(hood.moveCommand(false));
    controller.yButton.whileTrue(ShooterCommands.AimToHub(turret, () -> drive.getPose()));
    controller.aButton.whileTrue(
        ShooterCommands.AimToHub(turret, flywheel, hood, () -> drive.getPose(), 65));

    // Climber Binds
    controller.povUp.whileTrue(climber.upCommand());
    controller.povDown.whileTrue(climber.downCommand());

    // Intake Binds
    controller.xButton.whileTrue(CommandFactory.intakeCommand(intake, hopper));
    controller.bButton.whileTrue(CommandFactory.outtakeCommand(intake, hopper));
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
                            0.13, -0.2, new Rotation2d(turret.currentPosition * 2 * Math.PI))))
            .plus(new Transform3d(0, 0, 0.3, new Rotation3d())));
    Logger.recordOutput(
        "Hood/simulatedPose",
        new Pose3d(
                driveSimulation
                    .getSimulatedDriveTrainPose()
                    .plus(
                        new Transform2d(
                            0.13, -0.2, new Rotation2d(turret.currentPosition * 2 * Math.PI))))
            .plus(
                new Transform3d(0, 0, 0.3, new Rotation3d(0, hood.hoodAngle * Math.PI / 180, 0))));
    Logger.recordOutput(
        "Turret/targetPose",
        new Pose3d(
                driveSimulation
                    .getSimulatedDriveTrainPose()
                    .plus(
                        new Transform2d(
                            0.13, -0.2, new Rotation2d(turret.targetPosition * 2 * Math.PI))))
            .plus(new Transform3d(0, 0, 0.3, new Rotation3d())));
  }
}
