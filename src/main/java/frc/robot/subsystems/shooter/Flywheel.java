package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;

public class Flywheel extends SubsystemBase {
  private static TalonFX flywheelMotor;
  private double motorSpeed;

  public Flywheel(int flywheelMotorID, Mode state) {
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
    return Commands.run(() -> stop(), this).withName("stop flywheel");
  }

  public Command shootCommand() {
    return Commands.run(() -> shoot(), this).withName("shoot flywheel");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {}
}
