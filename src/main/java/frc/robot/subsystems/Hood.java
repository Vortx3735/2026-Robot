// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  /*define objects and variables here (e.g. motors, sensors, variables)
   *for example:
  public final Motor motor1;
   */
  private final TalonFX motor;
  private final CANcoder canCoder;
  /*initialize subsystem objects in constructor
   *for good practice, pass in any constants through the constructor
   */
  public Hood(int motorId, int canCoderId) {
    motor = new TalonFX(motorId);
    canCoder = new CANcoder(canCoderId);
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.set(0);
  }

  public Command runCommand(double speed) {
    return new RunCommand(() -> run(speed), this);
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this);
  }

  public double getHoodAngleDegrees() {
    return canCoder.getAbsolutePosition().getValueAsDouble() * 360.0;
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
