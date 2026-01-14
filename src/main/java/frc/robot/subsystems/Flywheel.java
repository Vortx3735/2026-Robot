package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  public static TalonFX flywheelMotor;

  public Flywheel(int flywheelMotorID) {
    flywheelMotor = new TalonFX(flywheelMotorID);

  }

  public void setFlywheelSpeed(double speed) {
    flywheelMotor.set(speed);
  }

}
