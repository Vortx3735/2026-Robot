package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private static TalonFX flywheelMotor;
  private double motorSpeed;
  final DoubleEntry flywheelMotorSpeedEntry;

  public Flywheel(int flywheelMotorID, Mode state) {
    flywheelMotor = new TalonFX(flywheelMotorID);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable intakeTable = inst.getTable("Flywheel");
    flywheelMotorSpeedEntry = intakeTable.getDoubleTopic("flywheelMotorSpeed").getEntry(0);
    flywheelMotorSpeedEntry.set(1);
  }

  public void setFlywheelSpeed(double speed) {
    motorSpeed = speed;
  }

  public double getFlywheelSpeed() {
    return motorSpeed;
  }

  public void shoot() {
    flywheelMotor.set(-motorSpeed);
  }

  public void stop() {
    flywheelMotor.set(0);
  }

  public Command stopCommand() {
    return Commands.run(() -> stop(), this).withName("stop flywheel");
  }

  public Command shootCommand() {
    return this.run(
            () -> {
              setFlywheelSpeed(flywheelMotorSpeedEntry.getAsDouble());
              this.shoot();
            })
        .withName("shoot flywheel");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    Logger.recordOutput("Flywheel/simulatedVoltage", flywheelMotor.getSimState().getMotorVoltage());
  }
}
