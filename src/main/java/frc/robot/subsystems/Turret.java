package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  public static TalonFX turretMotor;
  private double turretPosition;

  public Turret(int turretMotorID) {
    turretMotor = new TalonFX(turretMotorID);
  }

  public void setTurretSpeed(double speed) {
    turretMotor.set(speed);
  }

  public Command moveTurretRight(double speed) {
    return runOnce(() -> setTurretSpeed(speed));
  }

  public void changeTurretPosition(double position) {
    if (turretPosition < position) {
      setTurretSpeed(0.5);
    } else if (turretPosition > position) {
      setTurretSpeed(-0.5);
    } else {
      setTurretSpeed(0);
    }
  }

  public double getTurretPosition() {
    return turretPosition;
  }

  public void stop() {
    turretMotor.set(0);
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
