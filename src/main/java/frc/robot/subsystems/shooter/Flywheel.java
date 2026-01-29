package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;

public class Flywheel extends SubsystemBase {
  private static TalonFX flywheelMotor;
  private double motorSpeed;
  private NetworkTableInstance inst;
  private DoubleTopic dblTopic;
  public final DoubleEntry velocityEntry;
  public double simFlywheelVelocity = 0; // rad/sec

  public Flywheel(int flywheelMotorID, Mode state) {
    flywheelMotor = new TalonFX(flywheelMotorID);

    // Config PID/MotionMagic
    // Needed because we config PID and MotionMagic
    var talonFXConfigs = new TalonFXConfiguration();

    // Config PID
    var slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kS = 0.1; // Output to overcome static friction
    slot0Configs.kV = 0.12; // Output per unit of requested velocity
    slot0Configs.kP = 0.11; // Proportional
    slot0Configs.kI = 0; // Integral
    slot0Configs.kD = 0; // Derivative

    // Config MotionMagic
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400;
    motionMagicConfigs.MotionMagicJerk = 4000;

    flywheelMotor.getConfigurator().apply(talonFXConfigs);

    // Configure sim velocity logging
    inst = NetworkTableInstance.getDefault();
    dblTopic = inst.getDoubleTopic("Flywheel/SimulatedFlywheelVelocity");
    velocityEntry = dblTopic.getEntry(0.0);
    velocityEntry.set(simFlywheelVelocity);

    // Config motor sim state if mode is sim
    if (state == Mode.SIM) {
      var talonFXSim = flywheelMotor.getSimState();
      talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }
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
