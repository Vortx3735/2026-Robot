package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tunnel extends SubsystemBase {
  private final TalonFX tunnelMotor;
  private double speed = 0.25;

  // Network Table Entry
  final DoubleEntry tunnelMotorSpeedEntry;

  public Tunnel(int tunnelMotorId) {
    tunnelMotor = new TalonFX(tunnelMotorId);

    // Tunnel Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable tunnelTable = inst.getTable("Tunnel");
    tunnelMotorSpeedEntry = tunnelTable.getDoubleTopic("tunnelMotorSpeed").getEntry(0);
    tunnelMotorSpeedEntry.set(1);
  }

  public double getSpeed() {
    return speed;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void stopTunnel() {
    tunnelMotor.set(0);
  }

  public void runTunnel() {
    tunnelMotor.set(speed);
  }

  public Command tunnelCommand() {
    return this.run(
            () -> {
              setSpeed(tunnelMotorSpeedEntry.getAsDouble());
              runTunnel();
            })
        .withName("run tunnel");
  }

  public Command stopCommand() {
    return this.run(
            () -> {
              stopTunnel();
            })
        .withName("stop tunnel");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
