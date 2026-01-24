package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("Elastic").getSubTable("Flywheel");
  private final NetworkTableEntry ntActualRPM = table.getEntry("ActualRPM");
  private final NetworkTableEntry ntTargetRPM = table.getEntry("TargetRPM");
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

  @Override
  public void periodic() {
    publishTelemetry();
    readDashboardControls();
  }

  private void readDashboardControls() {
    // Read the target RPM from the Elastic slider
    motorSpeed = ntTargetRPM.getDouble(0);
  }

  private void publishTelemetry() {

    // Read the current flywheel speed from the encoder
    double currentRPM = motorSpeed;

    // Publish the measured speed to Elastic
    ntActualRPM.setDouble(currentRPM);
  }
}
