// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import stuff up here
// for example:
// import com.aaronsFavWebsite.motorLibrary.Motor;

public class Indexer extends SubsystemBase {
  /*define objects and variables here (e.g. motors, sensors, variables)
   *for example:
  public final Motor motor1;
   */
  private final TalonFX motor;

  /*initialize subsystem objects in constructor
   *for good practice, pass in any constants through the constructor
   */
  public Indexer(int motorId) {
    motor = new TalonFX(motorId);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
