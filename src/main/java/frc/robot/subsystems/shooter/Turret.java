package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private static final double kGearRatio = (11.0 * 10.0) / (50.0 * 83.0);
  private static final double kMOI = 0.0117; // kg*m^2

  private final TalonFX turretMotor;
  // private final CANcoder canCoder; // unsure if will be added yet

  final DoubleEntry turretSpeedEntry;
  final DoubleEntry turretPositionEntry;

  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), kMOI, 1 / kGearRatio),
          DCMotor.getKrakenX44(1));

  public double currentPosition; // rotations
  public double targetPosition = 0;
  public double turretVelocity = 0.0; // rotations per second (mechanism)
  private static final double kTurretPositionTolerance = 0.05; // rotations
  private static final double kTurretVelocityToleranceRps = 0.5; // rps
  private final boolean isSim;
  // Precomputed simulated voltage for deterministic sim behavior when setPositionPID is called
  private double simulatedInputVoltage = 0.0;

  public Turret(int turretMotorID, /*int canCoderId,*/ Mode state) {
    turretMotor = new TalonFX(turretMotorID);
    isSim = state == Mode.SIM;
    // canCoder = new CANcoder(canCoderId); // unsure if will be added yet

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.36;
    // slot0Configs.kV = 0.11931;
    // slot0Configs.kS = 0.0060924;
    // slot0Configs.kA =
    //     1
    //         / (kGearRatio
    //             * DCMotor.getKrakenX44(1).KtNMPerAmp
    //             / (DCMotor.getKrakenX44(1).rOhms
    //                 * kMOI)); // An acceleration of 1 rps/s requires 0.01 V output
    // slot0Configs.kV =
    //     (kGearRatio
    //             * kGearRatio
    //             * DCMotor.getKrakenX44(1).KtNMPerAmp
    //             / (DCMotor.getKrakenX44(1).KvRadPerSecPerVolt
    //                 * DCMotor.getKrakenX44(1).rOhms
    //                 * kMOI))
    //         * slot0Configs.kA; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 9; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.01; // A velocity error of 1 rps results in 0.1 V output

    // Slow values for testing
    // slot0Configs.kP = 1.15;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        3 * kGearRatio; // target cruise velocity of 3 rps after gearing
    motionMagicConfigs.MotionMagicAcceleration =
        200; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (90.0 / 360.0) / kGearRatio;
    talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -(90.0 / 360.0) / kGearRatio;

    turretMotor.getConfigurator().apply(talonFXConfigs);
    turretMotor.setNeutralMode(NeutralModeValue.Coast);
    // turretMotor.setPosition(0);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Subsystems/Turret");
    turretSpeedEntry = table.getDoubleTopic("turretSpeed").getEntry(0);
    turretSpeedEntry.set(0.1);
    turretPositionEntry = table.getDoubleTopic("turretPosition(rotations)").getEntry(0);
    turretPositionEntry.set(0);
    // configure talonfx sim state if the mode is sim
    if (state == Mode.SIM) {
      var talonFXSim = turretMotor.getSimState();
      talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
      talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }
  }

  public void setVoltage(double voltage) {
    VoltageOut request = new VoltageOut(voltage);
    turretMotor.setControl(request);
  }

  public double getCurrentPosition() {
    // Prefer simulated field in simulation; otherwise read from motor sensor
    if (isSim) {
      return currentPosition;
    }
    return turretMotor.getRotorPosition().getValueAsDouble() * kGearRatio;
  }

  public void setPositionPID(double rotations) {
    // create a Motion Magic request, voltage output
    // if (Math.abs(turretPosition - rotations) > error) {
    // final MotionMagicVoltage m_request = new MotionMagicVoltage(rotations / kGearRatio);
    final PositionVoltage m_request = new PositionVoltage(rotations / kGearRatio);
    turretMotor.setControl(m_request);
    // }
    targetPosition = rotations;
    // Precompute a conservative simulated input to drive the DCMotorSim when running tests.
    double sign = Math.signum(rotations - currentPosition);
    simulatedInputVoltage = Math.max(-12.0, Math.min(12.0, sign * 6.0));
  }

  public boolean isFinished() {
    double posErr = Math.abs(targetPosition - currentPosition);
    if (posErr > kTurretPositionTolerance) {
      return false;
    }
    if (isSim) {
      return Math.abs(turretVelocity) < kTurretVelocityToleranceRps;
    }
    return true;
  }

  public void set(double s) {
    turretMotor.set(s);
  }

  public void stop() {
    turretMotor.set(0);
  }

  public void zero() {
    turretMotor.setPosition(0);
  }

  // command factories / command helpers
  public Command moveCommand(boolean reversed) {
    return this.run(
            () -> {
              if (reversed) {
                this.set(-turretSpeedEntry.get());
              } else {
                this.set(turretSpeedEntry.get());
              }
            })
        .withName("Move Turret");
  }

  public Command setPositionPIDCommand(double rotations) {
    double timeoutSeconds = 5.0; // safety timeout for turret movement
    var timeoutNotifier =
        Commands.waitSeconds(timeoutSeconds)
            .andThen(
                () -> {
                  Logger.recordOutput("Turret/SetPositionTimeout", 1.0);
                  DriverStation.reportWarning(
                      "Turret.setPositionPIDCommand: timed out waiting for turret to reach "
                          + rotations
                          + " rotations",
                      false);
                });
    return Commands.sequence(
            runOnce(() -> setPositionPID(rotations)), Commands.waitUntil(this::isFinished))
        .raceWith(timeoutNotifier)
        .withName("Set Turret Position PID");
  }

  public Command setPositionPIDCommandManualSetpoint() {
    return run(() -> setPositionPID(turretPositionEntry.getAsDouble()))
        .withName("Set Turret Position PID (manual setpoint)");
  }

  public Command stopCommand() {
    return run(() -> stop()).withName("Stop Turret");
  }

  @Override
  public void simulationPeriodic() {
    var talonFXSim = turretMotor.getSimState();
    // var canCoderSim = canCoder.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(12);

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();
    double voltageIn = 0.0;
    try {
      voltageIn = motorVoltage.in(Volts);
    } catch (Exception e) {
      // ignore
    }
    // If the Talon sim reports very small voltage, prefer the precomputed simulatedInputVoltage
    if (Math.abs(voltageIn) < 0.5 && Math.abs(simulatedInputVoltage) > 1e-6) {
      voltageIn = simulatedInputVoltage;
    }
    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(voltageIn);
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    try {
      talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(1.0 / kGearRatio));
      talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(1.0 / kGearRatio));
    } catch (Exception e) {
      // ignore when simulator environment doesn't allow setting rotor directly
    }

    // apply stuff to CANCoder
    /*canCoderSim.setRawPosition(m_motorSimModel.getAngularPosition());
    canCoderSim.setVelocity(m_motorSimModel.getAngularVelocity());*/

    currentPosition = m_motorSimModel.getAngularPosition().in(Units.Rotations);
    turretVelocity =
        m_motorSimModel.getAngularVelocity().in(edu.wpi.first.units.Units.RotationsPerSecond);
    Logger.recordOutput("Turret/TargetPosition", targetPosition);
    Logger.recordOutput("Turret/SimulatedTurretPosition", currentPosition);
    // Clear simulated input when near target
    if (Math.abs(targetPosition - currentPosition) < 0.05) {
      simulatedInputVoltage = 0.0;
    }
  }

  @Override
  public void periodic() {
    // When not sim, read from the motor; otherwise simulationPeriodic handles currentPosition
    if (!isSim) {
      currentPosition = turretMotor.getRotorPosition().getValueAsDouble() * kGearRatio;
      try {
        turretVelocity = turretMotor.getRotorVelocity().getValueAsDouble() * kGearRatio;
      } catch (Exception e) {
        // ignore
      }
    }
    Logger.recordOutput("Turret/currentPostion(rotations)", currentPosition);
    Logger.recordOutput("Turret/targetPostion(rotations)", targetPosition);
  }
}
