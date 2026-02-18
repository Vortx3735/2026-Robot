// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;

public class Hood extends SubsystemBase {
  /*define objects and variables here (e.g. motors, sensors, variables)
   *for example:
  public final Motor motor1;
   */
  private final TalonFX motor;

  public Hood(int motorId, int canCoderId, Mode state) {
    motor = new TalonFX(motorId);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  private void stop() {
    motor.set(0);
  }

  public Command moveCommand(double speed) {
    return new RunCommand(() -> setSpeed(speed), this).withName("run hood");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop hood");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {}
}
