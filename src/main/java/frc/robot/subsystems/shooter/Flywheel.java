package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private static TalonFX flywheelMotor;
  private double motorSpeed;

  public Flywheel(int flywheelMotorID) {
    flywheelMotor = new TalonFX(flywheelMotorID);
  }

  public void setFlywheelSpeed(double speed) {
    motorSpeed = speed;
  }

  public double getFlywheelSpeed() {
    return motorSpeed;
  }

  public void shoot() {
    flywheelMotor.set(motorSpeed);
  }

  public void stop() {
    flywheelMotor.set(0);
  }

  public Command stopCommand() {
    return run(() -> stop());
  }

  public Command shootCommand() {
    return run(() -> shoot());
  }
}
