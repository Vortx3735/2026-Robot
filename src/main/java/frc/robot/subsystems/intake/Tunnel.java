// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.networktables.DoubleEntry;
// NetworkTable imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Tunnel extends SubsystemBase {
  private final TalonFX bottomTunnelMotor;
  private final TalonFX topTunnelMotor;

  // Network Table Entry
  private final DoubleEntry topTunnelSpeedEntry;
  private final DoubleEntry bottomTunnelSpeedEntry;

  private double topTunnelSpeed;
  private double bottomTunnelSpeed;

  public Tunnel(int bottomTunnelId, int topTunnelId) {
    bottomTunnelMotor = new TalonFX(bottomTunnelId);
    topTunnelMotor = new TalonFX(topTunnelId);

    TalonFXConfiguration bottomMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration topMotorConfig = new TalonFXConfiguration();
    bottomMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomTunnelMotor.getConfigurator().apply(bottomMotorConfig);
    topTunnelMotor.getConfigurator().apply(topMotorConfig);

    // Configure followers: roller follows tunnel (opposed), belt follows tunnel (same)
    // Tunnel Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable tunnelTable = inst.getTable("Tunnel");
    bottomTunnelSpeedEntry = tunnelTable.getDoubleTopic("bottomTunnelSpeed").getEntry(1);
    topTunnelSpeedEntry = tunnelTable.getDoubleTopic("topTunnelSpeed").getEntry(1);
    bottomTunnelSpeedEntry.set(1);
    topTunnelSpeedEntry.set(1);
  }

  public void setTunnelSpeed(double bottomTunnelSpeed, double topTunnelSpeed) {
    this.bottomTunnelSpeed = bottomTunnelSpeed;
    this.topTunnelSpeed = topTunnelSpeed;
  }

  public double getBottomTunnelSpeed() {
    return bottomTunnelSpeed;
  }

  public void run(Boolean inverted) {
    if (inverted) {
      bottomTunnelMotor.set(-bottomTunnelSpeed);
      topTunnelMotor.set(-topTunnelSpeed);
    } else {
      bottomTunnelMotor.set(bottomTunnelSpeed);
      topTunnelMotor.set(topTunnelSpeed);
    }
  }

  public void stop() {
    bottomTunnelMotor.set(0);
    topTunnelMotor.set(0);
  }

  public Command runTunnelCommand(Boolean inverted) {
    // Execute setTunnelSpeed AND set the motor every loop
    return run(() -> {
          setTunnelSpeed(bottomTunnelSpeedEntry.getAsDouble(), topTunnelSpeedEntry.getAsDouble());
          run(inverted); // Ensure the motor is actually updated
        })
        .withName("run tunnel");
  }

  public Command stopCommand() {
    return this.run(() -> stop()).withName("stop tunnel");
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Tunnel/bottomTunnelSpeed", bottomTunnelSpeed);
    Logger.recordOutput("Tunnel/topTunnelSpeed", topTunnelSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Logger.recordOutput(
        "Tunnel/simulatedVoltage1", bottomTunnelMotor.getSimState().getMotorVoltage());
    Logger.recordOutput("Tunnel/simulatedVoltage2", topTunnelMotor.getSimState().getMotorVoltage());
  }
}
