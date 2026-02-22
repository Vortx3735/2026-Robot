package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.networktables.DoubleEntry;
// NetworkTable imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final TalonFX indexerMotor;
  private final TalonFX rollerMotor;
  private final TalonFX beltMotor;

  private double indexerSpeed;
  private double rollerSpeed;

  // Network Table Entry
  final DoubleEntry indexerSpeedEntry;
  final DoubleEntry rollerSpeedEntry;

  public Indexer(int indexerID, int rollerID, int beltID) {
    indexerMotor = new TalonFX(indexerID);
    rollerMotor = new TalonFX(rollerID);
    beltMotor = new TalonFX(beltID);

    // Configure followers: roller follows indexer (opposed), belt follows indexer (same)
    beltMotor.setControl(new Follower(rollerID, MotorAlignmentValue.Opposed));
    // Indexer Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable indexerTable = inst.getTable("Indexer");
    indexerSpeedEntry = indexerTable.getDoubleTopic("indexerSpeed").getEntry(0);
    indexerSpeedEntry.set(0.7);
    rollerSpeedEntry = indexerTable.getDoubleTopic("rollerSpeed").getEntry(0);
    rollerSpeedEntry.set(0.7);
  }

  public void setIndexerSpeed(double indexerSpeed, double rollerSpeed) {
    this.indexerSpeed = indexerSpeed;
    this.rollerSpeed = rollerSpeed;
  }

  public double getIndexerSpeed() {
    return indexerSpeed;
  }

  public void run(Boolean inverted) {
    if (inverted) {
      indexerMotor.set(-indexerSpeed);
      rollerMotor.set(-rollerSpeed);
    } else {
      indexerMotor.set(indexerSpeed);
      rollerMotor.set(rollerSpeed);
    }
  }

  public void stop() {
    indexerMotor.set(0);
    rollerMotor.set(0);
  }

  public Command runIndexerCommand(Boolean inverted) {
    // Execute setIndexerSpeed AND set the motor every loop
    return run(() -> {
          setIndexerSpeed(indexerSpeedEntry.getAsDouble(), rollerSpeedEntry.getAsDouble());
          run(inverted); // Ensure the motor is actually updated
        })
        .withName("run indexer");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop indexer");
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Logger.recordOutput("Indexer/simulatedVoltage1", indexerMotor.getSimState().getMotorVoltage());
    Logger.recordOutput("Indexer/simulatedVoltage2", rollerMotor.getSimState().getMotorVoltage());
  }
}
