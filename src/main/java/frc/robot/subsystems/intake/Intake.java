// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.networktables.DoubleEntry;
// NetworkTable imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {

  private final TalonFX intakeMotor;
  private LoggedNetworkNumber supplyCurrentLimit =
      new LoggedNetworkNumber("Subsystems/Intake/supplyCurrentLimit", 15);
  // Holds the supply current limit that's currently applied so we can compare it to a new one
  private double curSupplyCurrentLimit = supplyCurrentLimit.get();

  // Network Table Entry
  final DoubleEntry intakeSpeedEntry;

  public Intake(int motorId) {
    intakeMotor = new TalonFX(motorId);

    // Intake Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable intakeTable = inst.getTable("Subsystems/Intake");
    intakeSpeedEntry = intakeTable.getDoubleTopic("intakeSpeed").getEntry(0);
    intakeSpeedEntry.set(1);
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var currentLimits = intakeConfig.CurrentLimits;

    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit.get();

    intakeMotor.getConfigurator().apply(intakeConfig);
  }

  public double getIntakeSpeed() {
    return intakeSpeedEntry.get();
  }

  public double getIntakeCurrentRPS() {
    return intakeMotor.getRotorVelocity().getValueAsDouble();
  }
  // Invert true is outtake. false is intake
  public void run(Boolean inverted) {
    if (inverted) {
      // outtake
      intakeMotor.set(-getIntakeSpeed());
    } else {
      // intake
      intakeMotor.set(getIntakeSpeed());
    }
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public Command intakeCommand() {
    return new RunCommand(() -> run(false), this).withName("intake intake");
  }

  public Command outtakeCommand() {
    return new RunCommand(() -> run(true), this).withName("outtake intake");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop intake");
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Intake/statorCurrent", intakeMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Intake/supplyCurrent", intakeMotor.getSupplyCurrent().getValueAsDouble());

    // Read new current limit from AdvantageScope
    double newSupplyCurrentLimit = supplyCurrentLimit.get();

    // Update supply current if it was changed
    if (newSupplyCurrentLimit != curSupplyCurrentLimit) {
      CurrentLimitsConfigs config = new CurrentLimitsConfigs();
      config.SupplyCurrentLimit = newSupplyCurrentLimit;
      intakeMotor.getConfigurator().apply(config);

      // Change the currently set supply current limit so that we know it was changed
      curSupplyCurrentLimit = newSupplyCurrentLimit;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Logger.recordOutput("Intake/simulatedVoltage", intakeMotor.getSimState().getMotorVoltage());
  }
}
