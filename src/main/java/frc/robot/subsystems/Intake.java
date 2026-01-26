// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final TalonFX motor;
  private double speed = 0.25;

  public Intake(int motorId) {
    motor = new TalonFX(motorId);
  }

  public double getSpeed() {
    return speed;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void stopIntake() {
    // Stop motor
    motor.set(0);
  }

  public void intake() {
    motor.set(speed);
  }

  public Command intakeCommand() {
    return this.run(() -> this.intake()).withName("run intake");
  }

  public Command stopCommand() {
    return this.run(() -> this.stopIntake()).withName("stop intake");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
