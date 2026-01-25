package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  public static TalonFX turretMotor;
  public double turretPosition;
  private static final double kGearRatio = 10.0;
  private static final double kMOI = 0.001; // kg*m^2
  public double targetRotations = 0;
  private final double error = 0.005;
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), kMOI, kGearRatio),
          DCMotor.getKrakenX60(1));

  public Turret(int turretMotorID, Mode state) {
    turretMotor = new TalonFX(turretMotorID);

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = 0.1; // Add 0.25 V output to overcome static friction
    // slot0Configs.kV = 0.1178; // A velocity target of 1 rps results in 0.12 V output
    // slot0Configs.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kA =
        1
            / (kGearRatio
                * DCMotor.getKrakenX60(1).KtNMPerAmp
                / (DCMotor.getKrakenX60(1).rOhms
                    * kMOI)); // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kV =
        (kGearRatio
                * kGearRatio
                * DCMotor.getKrakenX60(1).KtNMPerAmp
                / (DCMotor.getKrakenX60(1).KvRadPerSecPerVolt
                    * DCMotor.getKrakenX60(1).rOhms
                    * kMOI))
            * slot0Configs.kA; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 2.075; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.065; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        3 * kGearRatio; // target cruise velocity of 3 rps after gearing
    motionMagicConfigs.MotionMagicAcceleration =
        200; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    turretMotor.getConfigurator().apply(talonFXConfigs);

    // configure talonfx sim state if the mode is sim
    if (state == Mode.SIM) {
      var talonFXSim = turretMotor.getSimState();
      talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
      talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }
  }

  public void setTurretSpeed(double speed) {
    turretMotor.set(speed);
  }

  public void setVoltage(double voltage) {
    VoltageOut request = new VoltageOut(voltage);
    turretMotor.setControl(request);
  }

  public Command moveCommand(double speed) {
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

  public void setPositionPID(double rotations) {
    // create a Motion Magic request, voltage output
    // if (Math.abs(turretPosition - rotations) > error) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(rotations * kGearRatio);
    // final PositionVoltage m_request = new PositionVoltage(rotations * kGearRatio);
    turretMotor.setControl(m_request);
    // }
    targetRotations = rotations;
  }

  public Command setPositionPIDCommand(double rotations) {
    return runOnce(() -> setPositionPID(rotations));
  }

  public double getTurretPosition() {
    return turretPosition;
  }

  public void stop() {
    turretMotor.set(0);
  }

  public Command stopCommand() {
    return run(() -> stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    var talonFXSim = turretMotor.getSimState();

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

    turretPosition = m_motorSimModel.getAngularPosition().in(Units.Rotations);
    Logger.recordOutput("Turret/TargetPosition", targetRotations);
    Logger.recordOutput("Turret/SimulatedTurretPosition", turretPosition);
  }
}
