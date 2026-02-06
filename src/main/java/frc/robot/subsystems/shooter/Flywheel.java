package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private static TalonFX flywheelMotor;
  private double motorSpeed;
  /*private NetworkTableInstance inst;
  private DoubleTopic dblTopic;
  public final DoubleEntry velocityEntry;*/
  public double simFlywheelVelocity = 0; // rad/sec
  public double targetVelocity = 0;
  private static final double kGearRatio = 10.0;
  private static final double kMOI = 0.001; // kg*m^2
  private double commandedPercent = 0;
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), kMOI, kGearRatio),
          DCMotor.getKrakenX44(1));

  public Flywheel(int flywheelMotorID) {
    flywheelMotor = new TalonFX(flywheelMotorID);

    // Config PID/MotionMagic
    // Needed because we config PID and MotionMagic
    var talonFXConfigs = new TalonFXConfiguration();

    // Config PID
    var slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kS = 0.1; // Output to overcome static friction
    slot0Configs.kV = 0.12; // Velocity target
    slot0Configs.kP = 0.11; // Proportional
    slot0Configs.kI = 0; // Integral
    slot0Configs.kD = 0; // Derivative

    // Config MotionMagic
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 400;
    motionMagicConfigs.MotionMagicJerk = 4000;

    flywheelMotor.getConfigurator().apply(talonFXConfigs);

    /*// Configure sim velocity logging
    inst = NetworkTableInstance.getDefault();
    dblTopic = inst.getDoubleTopic("Flywheel/SimulatedFlywheelVelocity");
    velocityEntry = dblTopic.getEntry(0.0);
    velocityEntry.set(simFlywheelVelocity);*/

    // Config motor sim state if mode is sim
    if (Constants.currentMode == Mode.SIM) {
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
    commandedPercent = motorSpeed;
    flywheelMotor.set(motorSpeed);
  }

  public void stop() {
    commandedPercent = 0;
    flywheelMotor.set(0);
  }

  public void setPositionPID(double velocity) {
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(velocity);

    flywheelMotor.setControl(m_request);
    targetVelocity = velocity;
  }

  public Command setPositionPIDCommand(double rotations) {
    return run(() -> setPositionPID(rotations)).withName("Set Flywheel Position PID");
  }

  public Command stopCommand() {
    return run(() -> stop()).withName("stop flywheel");
  }

  public Command shootCommand() {
    return run(() -> shoot()).withName("shoot flywheel");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    var talonFXSim = flywheelMotor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(12);

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(commandedPercent * 12);
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
    talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));

    simFlywheelVelocity = m_motorSimModel.getAngularVelocity().in(Units.RadiansPerSecond);
    Logger.recordOutput("Flywheel/SimulatedFlywheelVelocity", simFlywheelVelocity);
    Logger.recordOutput("Flywheel/targetVelocity", targetVelocity);
  }
}
