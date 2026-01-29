// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final TalonFX climberMotor;
  private double speed = 0.25;

  public Climber(int climberMotorID) {
    climberMotor = new TalonFX(climberMotorID);
  }

  public double getSpeed() {
    return speed;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void up() {
    climberMotor.set(speed);
  }

  public void down() {
    climberMotor.set(-speed);
  }

  public void stop() {
    climberMotor.set(0);
  }

  public Command upCommand() {

    return this.run(() -> this.up()).withName("climb up");
  }

  public Command downCommand() {

    return this.run(() -> this.down()).withName("climb down");
  }

  public Command stopCommand() {

    return this.runOnce(() -> this.stop()).withName("stop climber");
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
