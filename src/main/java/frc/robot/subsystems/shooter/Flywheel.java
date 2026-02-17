package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private static TalonFX flywheelMotor;
  private double motorSpeed;
  public double currentVelocity;
  private static final double kGearRatio = 1.0;
  private static final double kMOI = 0.001; // kg*m^2
  public double targetVelocity = 0;
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), kMOI, kGearRatio),
          DCMotor.getKrakenX60(1));

  public Flywheel(int flywheelMotorID, Mode state) {
    flywheelMotor = new TalonFX(flywheelMotorID);
    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25;
    slot0Configs.kV = 0.12;
    slot0Configs.kA =
        1
            / (kGearRatio
                * DCMotor.getKrakenX60(1).KtNMPerAmp
                / (DCMotor.getKrakenX60(1).rOhms
                    * kMOI)); // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 1; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 500; // Target acceleration of 100 rps/s
    motionMagicConfigs.MotionMagicJerk = 6000; // Target jerk of 6000 rps/s/s (0.1 seconds)

    flywheelMotor.getConfigurator().apply(talonFXConfigs);
    if (state == Mode.SIM) {
      var talonFXSim = flywheelMotor.getSimState();
      talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
      talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
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
    targetVelocity = 0;
  }

  public void setVelocityPID(double rps) {
    // create a Motion Magic request, voltage output
    // if (Math.abs(turretPosition - rotations) > error) {
    // final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(rps *
    // kGearRatio);
    final VelocityVoltage m_request = new VelocityVoltage(rps * kGearRatio);
    flywheelMotor.setControl(m_request);
    // }
    targetVelocity = rps;
  }

  public Command setVelocityPIDCommand(double rps) {
    return Commands.run(() -> setVelocityPID(rps), this).withName("Set flywheel velocity PID");
  }

  public Command holdSpeed() {
    return Commands.run(() -> setVelocityPID(targetVelocity), this)
        .withName("hold flywheel velocity");
  }

  public Command stopCommand() {
    return Commands.run(() -> stop(), this).withName("stop flywheel");
  }

  public Command shootCommand() {
    return Commands.run(() -> shoot(), this).withName("shoot flywheel");
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Flywheel/TargetVelocity(rps)", targetVelocity);
    Logger.recordOutput("Flywheel/SimulatedVelocity(rps)", currentVelocity);
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
    talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
    talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
    currentVelocity = m_motorSimModel.getAngularVelocity().in(RotationsPerSecond);
    Logger.recordOutput("Flywheel/TargetVelocity(rps)", targetVelocity);
    Logger.recordOutput("Flywheel/SimulatedVelocity(rps)", currentVelocity);
  }
}
