package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
// NetworkTable imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final TalonFX indexerMotor;
  private double motorSpeed;

  // Network Table Entry
  final DoubleEntry indexerMotorSpeedEntry;

  public Indexer(int motorId) {
    indexerMotor = new TalonFX(motorId);

    // Indexer Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable indexerTable = inst.getTable("Indexer");
    indexerMotorSpeedEntry = indexerTable.getDoubleTopic("indexerMotorSpeed").getEntry(0);
  }

  public void setIndexerSpeed(double speed) {
    motorSpeed = speed;
  }

  public double getIndexerSpeed() {
    return motorSpeed;
  }

  public void run() {
    indexerMotor.set(motorSpeed);
  }

  public void stop() {
    indexerMotor.set(0);
  }

  public Command runCommand(double speed) {
    return run(() -> setIndexerSpeed(speed)).withName("run indexer");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop indexer");
  }

  @Override
  public void periodic() {
    readDashboardControls();
    publishTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Read dashboard controls
  private void readDashboardControls() {
    setIndexerSpeed(indexerMotorSpeedEntry.get());
  }

  // Publish telemetry to Network Table
  private void publishTelemetry() {
    indexerMotorSpeedEntry.set(indexerMotor.get());
  }
}
