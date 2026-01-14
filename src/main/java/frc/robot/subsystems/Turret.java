package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  public static TalonFX turretMotor;

  public Turret(int turretMotorID) {
    turretMotor = new TalonFX(turretMotorID);
  }

  public void setTurretSpeed(double speed) {
    turretMotor.set(speed);
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
