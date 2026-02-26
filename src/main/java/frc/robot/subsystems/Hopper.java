package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
// NetworkTable imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final TalonFX hopperMotor;

  // Network Table Entry
  final DoubleEntry hopperSpeedEntry;

  private double hopperSpeed;

  public Hopper(int hopperID) {
    hopperMotor = new TalonFX(hopperID);

    // Limit current
    var currentLimit = new CurrentLimitsConfigs();

    currentLimit.StatorCurrentLimit = 30;
    currentLimit.StatorCurrentLimitEnable = true;

    hopperMotor.getConfigurator().apply(currentLimit);

    // Hopper Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable hopperTable = inst.getTable("Hopper");
    hopperSpeedEntry = hopperTable.getDoubleTopic("hopperSpeed").getEntry(0);
    hopperSpeedEntry.set(0.1);
  }

  public void setHopperSpeed(double hopperSpeed) {
    this.hopperSpeed = hopperSpeed;
  }

  public double getHopperSpeed() {
    return hopperSpeed;
  }

  public void run(Boolean inverted) {
    if (inverted) {
      hopperMotor.set(-hopperSpeed);
    } else {
      hopperMotor.set(hopperSpeed);
    }
  }

  public void stop() {
    hopperMotor.set(0);
  }

  public Command runHopperCommand(Boolean inverted) {
    // Execute setHopperSpeed AND set the motor every loop
    return run(() -> {
          setHopperSpeed(hopperSpeedEntry.getAsDouble());
          run(inverted); // Ensure the motor is actually updated
        })
        .withName("run hopper");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop hopper");
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Hopper/motorCurrent", hopperMotor.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Logger.recordOutput("Hopper/simulatedVoltage1", hopperMotor.getSimState().getMotorVoltage());
  }
}
