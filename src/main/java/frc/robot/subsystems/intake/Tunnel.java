package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.networktables.DoubleEntry;
// NetworkTable imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Tunnel extends SubsystemBase {
  private final TalonFX bottomTunnelMotor;
  private final TalonFX topTunnelMotor;

  // Network Table Entry
  final DoubleEntry topTunnelSpeedEntry;
  final DoubleEntry bottomTunnelSpeedEntry;
  private static final double maxCurrent = 1000;
  private LoggedNetworkNumber supplyCurrentLimit =
      new LoggedNetworkNumber("Subsystems/Tunnel/supplyCurrentLimit", 10);
  // Holds the supply current limit that's currently applied so we can compare it to a new one
  private double curSupplyCurrentLimit = supplyCurrentLimit.get();

  public Tunnel(int bottomTunnelId, int topTunnelId) {
    bottomTunnelMotor = new TalonFX(bottomTunnelId);
    topTunnelMotor = new TalonFX(topTunnelId);

    TalonFXConfiguration bottomMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration topMotorConfig = new TalonFXConfiguration();
    bottomMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // var bottomCurrentLimits = bottomMotorConfig.CurrentLimits;

    // bottomCurrentLimits.SupplyCurrentLimitEnable = true;
    // bottomCurrentLimits.SupplyCurrentLimit = 300;
    // bottomCurrentLimits.StatorCurrentLimitEnable = true;
    // bottomCurrentLimits.StatorCurrentLimit = 300;

    // var topCurrentLimit = bottomMotorConfig.CurrentLimits;

    // topCurrentLimits.SupplyCurrentLimitEnable = true;
    // topCurrentLimits.SupplyCurrentLimit = 300;
    // topCurrentLimits.StatorCurrentLimitEnable = true;
    // topCurrentLimits.StatorCurrentLimit = 300;

    bottomTunnelMotor.getConfigurator().apply(bottomMotorConfig);
    topTunnelMotor.getConfigurator().apply(topMotorConfig);

    // Configure followers: roller follows tunnel (opposed), belt follows tunnel (same)
    // Tunnel Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable tunnelTable = inst.getTable("Subsystems/Tunnel");
    bottomTunnelSpeedEntry = tunnelTable.getDoubleTopic("bottomTunnelSpeed").getEntry(1);
    topTunnelSpeedEntry = tunnelTable.getDoubleTopic("topTunnelSpeed").getEntry(1);
    bottomTunnelSpeedEntry.set(0.35);
    topTunnelSpeedEntry.set(0.35);
  }

  public double getTopTunnelSpeed() {
    return topTunnelSpeedEntry.get();
  }

  public double getBottomTunnelSpeed() {
    return bottomTunnelSpeedEntry.get();
  }

  public double getTopTunnelCurrentRPS() {
    return topTunnelMotor.getRotorVelocity().getValueAsDouble();
  }

  public double getBottomTunnelCurrentRPS() {
    return bottomTunnelMotor.getRotorVelocity().getValueAsDouble();
  }

  public void run(Boolean inverted) {
    if (inverted) {
      bottomTunnelMotor.set(-getBottomTunnelSpeed());
      topTunnelMotor.set(-getTopTunnelSpeed() * 0.9);
    } else {
      bottomTunnelMotor.set(getBottomTunnelSpeed());
      topTunnelMotor.set(getTopTunnelSpeed() * 0.9);
    }
  }

  public void stop() {
    bottomTunnelMotor.set(0);
    topTunnelMotor.set(0);
  }

  public Command intakeCommand() {
    return Commands.run(() -> run(false));
  }

  public Command outtakeCommand() {
    return new RunCommand(() -> run(true), this).withName("outtake tunnel");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop tunnel");
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Tunnel/bottomStatorCurrent",
    // bottomTunnelMotorMotor.getStatorCurrent().getValueAsDouble());
    // Logger.recordOutput("Tunnel/supplyCurrent",
    // topTunnelMotor.getStatorCurrent().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Logger.recordOutput(
        "Tunnel/simulatedVoltage1", bottomTunnelMotor.getSimState().getMotorVoltage());
    Logger.recordOutput("Tunnel/simulatedVoltage2", topTunnelMotor.getSimState().getMotorVoltage());
  }
}
