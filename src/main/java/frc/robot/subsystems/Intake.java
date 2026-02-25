// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
// NetworkTable imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final TalonFX intakeMotor;
  // Network Table Entry
  private final DoubleEntry intakeMotorSpeedEntry;
  private double speed = 0.25;

  public Intake(int motorId) {
    intakeMotor = new TalonFX(motorId);

    // Intake Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable intakeTable = inst.getTable("Intake");
    intakeMotorSpeedEntry = intakeTable.getDoubleTopic("intakeMotorSpeed").getEntry(0);
    intakeMotorSpeedEntry.set(0.5);
  }

  public double getSpeed() {
    return speed;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void stopIntake() {
    // Stop motor
    intakeMotor.set(0);
  }

  public void intake() {
    intakeMotor.set(speed);
  }

  public Command intakeCommand() {
    return this.run(
            () -> {
              setSpeed(intakeMotorSpeedEntry.getAsDouble());
              this.intake();
            })
        .withName("run intake");
  }

  public Command stopCommand() {
    return this.run(() -> this.stopIntake()).withName("stop intake");
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Logger.recordOutput("Intake/simulatedVoltage", intakeMotor.getSimState().getMotorVoltage());
  }
}
