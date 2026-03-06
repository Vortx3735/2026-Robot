package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Tunnel;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class Telemetry extends SubsystemBase {
  Drive drive;
  Vision vision;
  Flywheel flywheel;
  Hood hood;
  Turret turret;
  Hopper hopper;
  Intake intake;
  Tunnel tunnel;
  Climber climber;

  // Network Tables
  static final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  // Flywheel Network Table
  static final NetworkTable flywheelTable = inst.getTable("Subsystems/Flywheel");
  static final DoubleEntry flywheelSpeedEntry = flywheelTable.getDoubleTopic("flywheelSpeed").getEntry(1);
  flywheelSpeedEntry.set(0.75);

    // Hood Network Table
    NetworkTable hoodTable = inst.getTable("Subsystems/Hood");
    hoodSpeedEntry = hoodTable.getDoubleTopic("hoodSpeed").getEntry(0);
    hoodSpeedEntry.set(0.1);

    // Turret Network Table
    NetworkTable turretTable = inst.getTable("Subsystems/Turret");
    turretSpeedEntry = turretTable.getDoubleTopic("turretSpeed").getEntry(0);
    turretSpeedEntry.set(0.1);
    turretPositionEntry = turretTable.getDoubleTopic("turretPosition(rotations)").getEntry(0);
    turretPositionEntry.set(0);

    // Hopper Network Table
    NetworkTable hopperTable = inst.getTable("Subsystems/Hopper");
    hopperSpeedEntry = hopperTable.getDoubleTopic("hopperSpeed").getEntry(0);
    hopperSpeedEntry.set(0.3);

    // Intake Network Table
    NetworkTable intakeTable = inst.getTable("Subsystems/Intake");
    intakeSpeedEntry = intakeTable.getDoubleTopic("intakeSpeed").getEntry(0);
    intakeSpeedEntry.set(0.5);

    // Climber Network Table
    NetworkTable climberTable = inst.getTable("Subsystems/Climber");
    climberSpeedEntry = climberTable.getDoubleTopic("flywheelMotorSpeed").getEntry(0);
    climberSpeedEntry.set(0.25);

    // Tunnel Network Table
    NetworkTable tunnelTable = inst.getTable("Subsystems/Tunnel");
    bottomTunnelSpeedEntry = tunnelTable.getDoubleTopic("bottomTunnelSpeed").getEntry(1);
    topTunnelSpeedEntry = tunnelTable.getDoubleTopic("topTunnelSpeed").getEntry(1);
    bottomTunnelSpeedEntry.set(0.4);
    topTunnelSpeedEntry.set(0.4);


  public Telemetry(
      Drive drive,
      Vision vision,
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      Hopper hopper,
      Intake intake,
      Tunnel tunnel,
      Climber climber) {
    this.drive = drive;
    this.vision = vision;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.hopper = hopper;
    this.intake = intake;
    this.tunnel = tunnel;
    this.climber = climber;
  }


  @Override
  public void periodic() {
    // Drive logging
    Logger.recordOutput("Drive/pose", drive.getPose());
    // Vision Logging
    // Logger.recordOutput("VisionTest/hasTag", vision.hasTag);
    // Intake logging
    Logger.recordOutput("Tunnel/topCurrentRPS", tunnel.getTopTunnelCurrentRPS());
    Logger.recordOutput("Tunnel/bottomCurrentRPS", tunnel.getBottomTunnelCurrentRPS());
    Logger.recordOutput("Hopper/currentRPS", hopper.getHopperCurrentRPS());
    Logger.recordOutput("Intake/currentRPS", intake.getIntakeCurrentRPS());
    // Shooter logging
    Logger.recordOutput("Shooter/Turret/currentPosition(rotations)", turret.currentPosition);
    Logger.recordOutput("Shooter/Turret/targetPosition(rotations)", turret.targetPosition);
    Logger.recordOutput("Shooter/Hood/currentPosition", hood.hoodAngle);
    Logger.recordOutput("Shooter/Hood/targetPosition", hood.targetAngle);
    Logger.recordOutput("Shooter/Flywheel/currentRPS", flywheel.currentRPS);
    Logger.recordOutput("Shooter/Flywheel/targetRPS", flywheel.targetRPS);
  }
}
