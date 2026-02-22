package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;

public class Turret extends SubsystemBase {
  public static TalonFX turretMotor;
  final DoubleEntry turretMotorSpeedEntry;
  double speed;

  public Turret(int turretMotorID, Mode state) {
    turretMotor = new TalonFX(turretMotorID);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable intakeTable = inst.getTable("Turret");
    turretMotorSpeedEntry = intakeTable.getDoubleTopic("turretMotorSpeed").getEntry(0);
    turretMotorSpeedEntry.set(0.1);
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public void set(double s) {
    turretMotor.set(s);
  }

  public Command moveCommand(boolean reversed) {
    return this.run(
            () -> {
              setSpeed(turretMotorSpeedEntry.getAsDouble());
              if (reversed) {
                this.set(-speed);
              } else {
                this.set(speed);
              }
            })
        .withName("Move Turret");
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
