package frc.robot.subsystems;

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
    // hood -75.828
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
    Logger.recordOutput(
        "Shooter/Turret/currentPosition(rotations)", turret.getTurretCurrentPosition());
    Logger.recordOutput(
        "Shooter/Turret/targetPosition(rotations)", turret.getTurretTargetPosition());
    Logger.recordOutput("Shooter/Hood/currentPosition", hood.getHoodAngle());
    Logger.recordOutput("Shooter/Hood/targetPosition", hood.getHoodTargetAngle());
    Logger.recordOutput("Shooter/Flywheel/currentRPS", flywheel.getFlywheelCurrentRPS());
    Logger.recordOutput("Shooter/Flywheel/targetRPS", flywheel.getFlywheelTargetRPS());
    Logger.recordOutput("Shooter/isAtSpeed", flywheel.isAtSpeed());
  }
}
