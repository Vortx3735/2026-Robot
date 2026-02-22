// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  /*define objects and variables here (e.g. motors, sensors, variables)
   *for example:
  public final Motor motor1;
   */
  private final TalonFX motor;
  final DoubleEntry hoodMotorSpeedEntry;
  double speed;

  public Hood(int motorId, int canCoderId, Mode state) {
    motor = new TalonFX(motorId);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable intakeTable = inst.getTable("Hood");
    hoodMotorSpeedEntry = intakeTable.getDoubleTopic("hoodMotorSpeed").getEntry(0);
    hoodMotorSpeedEntry.set(0.1);
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void set(boolean reversed) {
    if (reversed) {
      motor.set(-speed);
    } else {
      motor.set(speed);
    }
  }

  private void stop() {
    motor.set(0);
  }

  public Command moveCommand(boolean reversed) {
    return this.run(
            () -> {
              setSpeed(hoodMotorSpeedEntry.getAsDouble());
              this.set(reversed);
            })
        .withName("move hood");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop hood");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    Logger.recordOutput("Hood/simulatedVoltage", motor.getSimState().getMotorVoltage());
  }
}
