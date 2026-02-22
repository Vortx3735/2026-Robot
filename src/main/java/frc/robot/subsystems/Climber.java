// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final TalonFX climberMotor1;
  private final TalonFX climberMotor2;
  private double speed = 0.25;
  final DoubleEntry flywheelMotorSpeedEntry;

  public Climber(int motorIdLeft, int motorIdRight) {
    climberMotor1 = new TalonFX(motorIdLeft);
    climberMotor2 = new TalonFX(motorIdRight);
    climberMotor2.setControl(new Follower(motorIdLeft, MotorAlignmentValue.Opposed));
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable intakeTable = inst.getTable("Intake");
    flywheelMotorSpeedEntry = intakeTable.getDoubleTopic("flywheelMotorSpeed").getEntry(0);
    flywheelMotorSpeedEntry.set(1);
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
            () -> {
              setSpeed(flywheelMotorSpeedEntry.getAsDouble());
              this.up();
            })
        .withName("run intake");
  }

  public Command downCommand() {

    return this.run(
            () -> {
              setSpeed(flywheelMotorSpeedEntry.getAsDouble());
              this.down();
            })
        .withName("run intake");
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
    Logger.recordOutput("Climber/simulatedVoltage", climberMotor1.getSimState().getMotorVoltage());
  }
}
