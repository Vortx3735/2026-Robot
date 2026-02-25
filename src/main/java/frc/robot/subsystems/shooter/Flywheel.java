package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private static final double kMOI = 0.001; // kg*m^2
  private static final double kMaxSpeed = 90; // Max speed in RPS

  private TalonFX flywheelMotor;
  public final DoubleEntry flywheelMotorSpeedEntry;

  private final BangBangController bangBangController = new BangBangController();
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), kMOI, 1),
          DCMotor.getKrakenX60(1));

  private double currentRPS;
  public double simulatedVelocity;
  public double targetRPS = 0;

  public Flywheel(int flywheelMotorID, Mode state) {
    flywheelMotor = new TalonFX(flywheelMotorID);
    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.31;
    slot0Configs.kV = 0.125;
    slot0Configs.kP = 0; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 500; // Target acceleration of 100 rps/s
    motionMagicConfigs.MotionMagicJerk = 6000; // Target jerk of 6000 rps/s/s (0.1 seconds)

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelMotor.getConfigurator().apply(talonFXConfigs);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable flywheelTable = inst.getTable("Flywheel");
    flywheelMotorSpeedEntry = flywheelTable.getDoubleTopic("flywheelMotorSpeed").getEntry(1);
    flywheelMotorSpeedEntry.set(0.9);
    if (state == Mode.SIM) {
      var talonFXSim = flywheelMotor.getSimState();
      talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
      talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }
  }

  public void stop() {
    flywheelMotor.set(0);
    targetRPS = 0;
  }

  public void setVelocityPID() {
    // final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(rps *
    // kGearRatio);
    targetRPS = flywheelMotorSpeedEntry.getAsDouble() * kMaxSpeed;
    final VelocityVoltage m_request = new VelocityVoltage(targetRPS);
    flywheelMotor.setControl(m_request);
    // flywheelMotor.set(bangBangController.calculate(currentRPS, targetRPS));
    // flywheelMotor.set(1);
  }

  public boolean isAtSpeed() {
    double tolerance = 3;
    return Math.abs(targetRPS - currentRPS) < tolerance;
  }

  public Command shootCommand() {
    return Commands.run(() -> setVelocityPID(), this).withName("shoot flywheel");
  }

  public Command stopCommand() {
    return Commands.run(() -> stop(), this).withName("stop flywheel");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentRPS = flywheelMotor.getRotorVelocity().getValueAsDouble();
    Logger.recordOutput("Flywheel/currentRPS", currentRPS);
    Logger.recordOutput("Flywheel/targetRPS", targetRPS);
  }

  @Override
  public void simulationPeriodic() {
    var talonFXSim = flywheelMotor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(12);

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();
    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    // talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
    // talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
    simulatedVelocity = m_motorSimModel.getAngularVelocity().in(RotationsPerSecond);
    Logger.recordOutput("Flywheel/TargetVelocity(rps)", targetRPS);
    Logger.recordOutput("Flywheel/SimulatedVelocity(rps)", simulatedVelocity);
  }
}
