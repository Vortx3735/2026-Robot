package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final TalonFX indexerMotor;
  private final TalonFX rollerMotor;
  private double indexerMotorSpeed;
  private double rollerMotorSpeed;

  // Network Table Entry
  final DoubleEntry indexerMotorSpeedEntry;

  public Indexer(int indexerID, int rollerID) {
    indexerMotor = new TalonFX(indexerID);
    rollerMotor = new TalonFX(rollerID);

    // Indexer Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable indexerTable = inst.getTable("Indexer");
    indexerMotorSpeedEntry = indexerTable.getDoubleTopic("indexerMotorSpeed").getEntry(0);
    indexerMotorSpeedEntry.set(1);
  }

  public void setIndexerSpeed(double speed) {
    indexerMotorSpeed = speed;
  }

  public void setRollerSpeed(double speed) {
    rollerMotorSpeed = speed;
  }

  public double getIndexerSpeed() {
    return indexerMotorSpeed;
  }

  public double getRollerSpeed() {
    return rollerMotorSpeed;
  }

  public void run(Boolean inverted) {
    if (inverted) {
      indexerMotor.set(-indexerMotorSpeed);
    } else {
      indexerMotor.set(indexerMotorSpeed);
    }
  }

  public void runIndexerMotor() {
    indexerMotor.set(indexerMotorSpeed);
  }

  public void runRollerMotor() {
    rollerMotor.set(rollerMotorSpeed);
  }

  public void stop() {
    indexerMotor.set(0);
    rollerMotor.set(0);
  }

  public Command runIndexerCommand(Boolean inverted) {
    // Execute setIndexerSpeed AND set the motor every loop
    return run(() -> {
          setIndexerSpeed(indexerMotorSpeedEntry.getAsDouble());
          run(inverted); // Ensure the motor is actually updated
        })
        .withName("run indexer");
  }

  public Command runIndexerMotorCommand() {
    return this.run(() -> this.runIndexerMotor()).withName("run indexer motor");
  }

  public Command runRollerMotorCommand() {
    return this.run(() -> this.runRollerMotor()).withName("run roller motor");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop indexer");
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
