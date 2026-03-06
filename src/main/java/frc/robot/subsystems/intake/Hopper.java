package frc.robot.subsystems.intake;

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

  public Hopper(int hopperID) {
    hopperMotor = new TalonFX(hopperID);
  }

  public double getHopperSpeed() {
    return hopperSpeedEntry.get();
  }

  public double getHopperCurrentRPS() {
    return hopperMotor.getRotorVelocity().getValueAsDouble();
  }

  // Invert true is outtake. false is intake
  public void run(Boolean inverted) {
    if (inverted) {
      // outtake
      hopperMotor.set(-getHopperSpeed());
    } else {
      // intake
      hopperMotor.set(getHopperSpeed());
    }
  }

  public void stop() {
    hopperMotor.set(0);
  }

  public Command intakeCommand() {
    return new RunCommand(() -> run(false), this).withName("intake hopper");
  }

  public Command outtakeCommand() {
    return new RunCommand(() -> run(true), this).withName("outtake hopper");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop hopper");
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Logger.recordOutput("Hopper/simulatedVoltage1", hopperMotor.getSimState().getMotorVoltage());
    Logger.recordOutput("Hopper/speed", hopperSpeedEntry.get());
  }
}
