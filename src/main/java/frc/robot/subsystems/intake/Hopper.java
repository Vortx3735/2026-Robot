package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class Hopper extends SubsystemBase {
  private static final double maxCurrent = 65; // for anti jam
  private final TalonFX hopperMotor;
  private LoggedNetworkNumber supplyCurrentLimit =
      new LoggedNetworkNumber("Subsystems/Hopper/supplyCurrentLimit", 15);
  // Holds the supply current limit that's currently applied so we can compare it to a new one
  private double curSupplyCurrentLimit = supplyCurrentLimit.get();

  // Network Table Entry
  final DoubleEntry hopperSpeedEntry;

  public Hopper(int hopperID) {
    hopperMotor = new TalonFX(hopperID);

    // Hopper Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable hopperTable = inst.getTable("Subsystems/Hopper");
    hopperSpeedEntry = hopperTable.getDoubleTopic("hopperSpeed").getEntry(0);
    hopperSpeedEntry.set(0.3);

    var talonFXConfigs = new TalonFXConfiguration();

    var currentLimits = talonFXConfigs.CurrentLimits;

    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit.get();

    hopperMotor.getConfigurator().apply(talonFXConfigs);
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

  public void runSlow(Boolean inverted) {
    if (inverted) {
      // outtake
      hopperMotor.set(-getHopperSpeed() + 0.08);
    } else {
      // intake
      hopperMotor.set(getHopperSpeed() - 0.08);
    }
  }

  public void stop() {
    hopperMotor.set(0);
  }

  public Command intakeCommand() {
    // if hopper motor is above max current, make it outtake instead (to prevent jamming)
    return Commands.either(
        Commands.sequence(
            Commands.deadline(Commands.waitSeconds(0.25), Commands.run(() -> run(true))),
            Commands.deadline(Commands.waitSeconds(0.25), Commands.run(() -> run(false)))),
        Commands.run(() -> run(false)),
        () -> hopperMotor.getStatorCurrent().getValueAsDouble() >= maxCurrent);
  }

  public Command intakeSlowCommand() {
    // if hopper motor is above max current, make it outtake instead (to prevent jamming)
    return Commands.either(
        Commands.sequence(
            Commands.deadline(Commands.waitSeconds(0.25), Commands.run(() -> run(true))),
            Commands.deadline(Commands.waitSeconds(0.25), Commands.run(() -> run(false)))),
        Commands.run(() -> runSlow(false)),
        () -> hopperMotor.getStatorCurrent().getValueAsDouble() >= maxCurrent - 5);
  }

  public Command outtakeCommand() {
    return new RunCommand(() -> run(true), this).withName("outtake hopper");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop hopper");
  }

  @Override
  public void periodic() {
    // Log currents
    Logger.recordOutput("Hopper/statorCurrent", hopperMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Hopper/supplyCurrent", hopperMotor.getSupplyCurrent().getValueAsDouble());

    // Read new current limit from AdvantageScope
    double newSupplyCurrentLimit = supplyCurrentLimit.get();

    // Update supply current if it was changed
    if (newSupplyCurrentLimit != curSupplyCurrentLimit) {
      CurrentLimitsConfigs config = new CurrentLimitsConfigs();
      config.SupplyCurrentLimit = newSupplyCurrentLimit;
      hopperMotor.getConfigurator().apply(config);

      // Change the currently set supply current limit so that we know it was changed
      curSupplyCurrentLimit = newSupplyCurrentLimit;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Logger.recordOutput("Hopper/simulatedVoltage1", hopperMotor.getSimState().getMotorVoltage());
    Logger.recordOutput("Hopper/speed", hopperSpeedEntry.get());
  }
}
