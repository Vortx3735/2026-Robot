package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;

public class Turret extends SubsystemBase {
  public static TalonFX turretMotor;

  public Turret(int turretMotorID, Mode state) {
    turretMotor = new TalonFX(turretMotorID);
  }

  public void setTurretSpeed(double speed) {
    turretMotor.set(speed);
  }

  public Command moveCommand(double speed) {
    return run(() -> setTurretSpeed(speed)).withName("Move Turret");
  }

  public void stop() {
    turretMotor.set(0);
  }

  public Command stopCommand() {
    return run(() -> stop()).withName("Stop Turret");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {}
}
