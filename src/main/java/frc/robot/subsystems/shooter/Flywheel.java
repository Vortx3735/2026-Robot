package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  private static TalonFX flywheelMotor;
  private double motorSpeed;
  private NetworkTableInstance inst;
  private DoubleTopic dblTopic;
  public final DoubleEntry velocityEntry;
  public double simFlywheelVelocity; // rad/sec

  public Flywheel(int flywheelMotorID) {
    flywheelMotor = new TalonFX(flywheelMotorID);
    inst = NetworkTableInstance.getDefault();
    dblTopic = inst.getDoubleTopic("Flywheel/SimulatedFlywheelVelocity");
    velocityEntry = dblTopic.getEntry(0.0);
    simFlywheelVelocity = 0;
    velocityEntry.set(simFlywheelVelocity);
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
    return run(() -> stop()).withName("stop flywheel");
  }

  public Command shootCommand() {
    return run(() -> shoot()).withName("shoot flywheel");
  }

  @Override
  public void simulationPeriodic() {
    simFlywheelVelocity = velocityEntry.get(0.0);
  }
}
