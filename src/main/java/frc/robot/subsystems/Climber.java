// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {

  private final TalonFX climberMotor1;
  private double speed = 0.25;

  public Climber(int motorId) {
    climberMotor1 = new TalonFX(motorId);
  }

  public double getSpeed() {
    return speed;
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void up() {
    climberMotor1.set(speed);
  }
  public void down() {
    climberMotor1.set(-speed);
  }
  public void stop() {
    climberMotor1.set(0);
  }
  public Command upCommand() {

    return this.run(
        () ->
  
            this.up());
  }
public Command downCommand() {

    return this.run(
        () ->
      
            this.down());
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
