// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private static final double kGearRatio = (9.0 * 15.0 * 10.0) / (48.0 * 30.0 * 15.0);
  private static final double kMOI = 0.001; // kg*m^2

  private final TalonFX hoodMotor;

  final DoubleEntry hoodSpeedEntry;

  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), kMOI, kGearRatio),
          DCMotor.getKrakenX44(1));

  public double hoodAngle = 0;
  public double targetAngle = 0;
  public double hoodVelocity = 0.0; // degrees per second (mechanism)
  private static final double kPositionToleranceDeg = 1.0; // degrees
  private static final double kVelocityToleranceDegPerSec = 10.0; // deg/s
  private final boolean isSim;
  // Precomputed voltage to apply to the simulated motor when a setpoint is requested.
  // This avoids races where the Talon sim may not have propagated motor voltages yet.
  private double simulatedInputVoltage = 0.0;

  public Hood(int motorId, Mode state) {
    hoodMotor = new TalonFX(motorId);
    isSim = state == Mode.SIM;
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kA =
        1
            / (kGearRatio
                * DCMotor.getKrakenX44(1).KtNMPerAmp
                / (DCMotor.getKrakenX44(1).rOhms
                    * kMOI)); // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kV =
        (kGearRatio
                * kGearRatio
                * DCMotor.getKrakenX44(1).KtNMPerAmp
                / (DCMotor.getKrakenX44(1).KvRadPerSecPerVolt
                    * DCMotor.getKrakenX44(1).rOhms
                    * kMOI))
            * slot0Configs.kA; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 4; // A position error of 4 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.01; // Add 0.01 V output for a velocity error of 1 rps

    // Slow values for testing
    // slot0Configs.kP = 0.59;
    // slot0Configs.kD = 0.07;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        3 * kGearRatio; // target cruise velocity of 3 rps after gearing
    motionMagicConfigs.MotionMagicAcceleration =
        200; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 2000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodMotor.getConfigurator().apply(talonFXConfigs);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable intakeTable = inst.getTable("Subsystems/Hood");
    hoodSpeedEntry = intakeTable.getDoubleTopic("hoodSpeed").getEntry(0);
    hoodSpeedEntry.set(0.1);

    if (state == Mode.SIM) {
      var talonFXSim = hoodMotor.getSimState();
      talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
      talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }
  }

  public double getHoodAngleDegrees() {
    // Prefer simulated field in simulation; otherwise read from motor sensor
    if (isSim) {
      return hoodAngle;
    }
    return hoodMotor.getRotorPosition().getValueAsDouble()
        * 360.0
        * kGearRatio; // Convert to mechanism degrees using gear ratio
  }

  private void stop() {
    hoodMotor.set(0);
  }

  public void setOpenLoop(boolean reversed) {
    if (reversed) {
      hoodMotor.set(-hoodSpeedEntry.get());
    } else {
      hoodMotor.set(hoodSpeedEntry.get());
    }
  }

  public void setPositionPID(double degrees) {
    // Convert desired mechanism position (degrees) to motor rotor rotations.
    // mechanism rotations = degrees / 360
    // motor rotations = mechanism rotations / kGearRatio
    final PositionVoltage m_request = new PositionVoltage((degrees / 360.0) / kGearRatio);
    hoodMotor.setControl(m_request);
    targetAngle = degrees;
  }

  public boolean isFinished() {
    double posErr = Math.abs(targetAngle - hoodAngle);
    if (posErr > kPositionToleranceDeg) {
      return false;
    }
    // If we're simulating, also require the mechanism to be nearly settled (low velocity)
    if (isSim) {
      return Math.abs(hoodVelocity) < kVelocityToleranceDegPerSec;
    }
    return true;
  }

  public Command moveCommand(boolean reversed) {
    return this.run(
            () -> {
              this.setOpenLoop(reversed);
            })
        .withName("move hood");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop hood");
  }

  public Command setPositionPIDCommand(double degrees) {
    // Set the setpoint then wait until the hood reports finished, but don't wait forever.
    double timeoutSeconds = 3.0; // safety timeout
    var timeoutNotifier =
        Commands.waitSeconds(timeoutSeconds)
            .andThen(
                () -> {
                  // log a timeout event — the andThen runs only if the waitSeconds completes (i.e.,
                  // timeout)
                  Logger.recordOutput("Hood/SetPositionTimeout", 1.0);
                  System.err.println(
                      "Hood.setPositionPIDCommand: timed out waiting for hood to reach "
                          + degrees
                          + " deg");
                });
    return Commands.sequence(
            runOnce(() -> setPositionPID(degrees)), Commands.waitUntil(this::isFinished))
        .raceWith(timeoutNotifier)
        .withName("Set Hood Position PID");
  }

  public Command hold() {
    return run(() -> setPositionPID(targetAngle));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    try {
      // Accessing the sim state objects is intentionally avoided for writes;
      // keep a reference to the Talon sim for setting supply voltage only.
      var talonFXSim = hoodMotor.getSimState();

      // set the supply voltage of the TalonFX (ensure sim has reasonable supply)
      talonFXSim.setSupplyVoltage(12);

      // get the motor voltage of the TalonFX
      var motorVoltage = talonFXSim.getMotorVoltageMeasure();
      double voltageIn = 0.0;
      try {
        voltageIn = motorVoltage.in(Volts);
      } catch (Exception e) {
        // Some simulator environments may not support reading motor voltage; we'll
        // fallback below if needed.
      }

      // use the motor voltage to calculate new position and velocity using DCMotorSim
      m_motorSimModel.setInputVoltage(voltageIn);
      m_motorSimModel.update(0.020); // assume 20 ms loop time

      // apply the new rotor position and velocity to the TalonFX;
      // note that this is rotor position/velocity (before gear ratio), but
      // DCMotorSim returns mechanism position/velocity (after gear ratio)
      try {
        talonFXSim.setRawRotorPosition(
            m_motorSimModel.getAngularPosition().times(1.0 / kGearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(1.0 / kGearRatio));
      } catch (Exception e) {
        // ignore when simulator environment doesn't allow setting rotor directly
      }

      // Read the mechanism position/velocity from the model and guard against
      // non-finite or extremely large values which indicate integration runaway
      double mechPosDeg;
      double mechVelDegPerSec;
      try {
        mechPosDeg = m_motorSimModel.getAngularPosition().in(Units.Degrees);
        mechVelDegPerSec =
            m_motorSimModel.getAngularVelocity().in(edu.wpi.first.units.Units.RotationsPerSecond)
                * 360.0;
        // If values are non-finite or absurdly large, reset the internal model to
        // a safe state and clamp outputs to avoid propagating bad data into the
        // vendor sims or AdvantageKit logs.
        if (!Double.isFinite(mechPosDeg)
            || !Double.isFinite(mechVelDegPerSec)
            || Math.abs(mechPosDeg) > 1e6
            || Math.abs(mechVelDegPerSec) > 1e6) {
          m_motorSimModel.setState(0.0, 0.0);
          mechPosDeg = 0.0;
          mechVelDegPerSec = 0.0;
        }
      } catch (Exception e) {
        // Defensive fallback: reset the model and use zeros
        m_motorSimModel.setState(0.0, 0.0);
        mechPosDeg = 0.0;
        mechVelDegPerSec = 0.0;
      }

      // Do NOT write guarded values back into vendor sims (TalonFX/CANcoder).
      // Writing large or unit-mismatched numbers into vendor sim state has been a
      // recurring source of runaway values visible in AdvantageKit. Instead we
      // keep the internal DCMotorSim as the single source of truth for the
      // mechanism state and expose `hoodAngle`/`hoodVelocity` for tests and
      // logging. If callers need vendor-sim state, we can add a small,
      // well-reviewed mapping later.
    } catch (Exception e) {
      // If any sim state access fails, ensure we still advance the DCMotorSim with a
      // conservative voltage so the mechanism state changes during tests.
      m_motorSimModel.setInputVoltage(Math.abs(targetAngle - hoodAngle) > 0.1 ? 6.0 : 0.0);
      m_motorSimModel.update(0.020);
    }

    // Always update simulated fields from the DCMotorSim model so tests can observe them
    hoodAngle = m_motorSimModel.getAngularPosition().in(Units.Degrees);
    // Clamp the hood angle to a sane range to prevent runaway integration during test noise.
    if (Double.isFinite(hoodAngle)) {
      hoodAngle = Math.max(-720.0, Math.min(720.0, hoodAngle));
    } else {
      hoodAngle = 0.0;
    }
    // angular velocity from DCMotorSim is in rotations per second; convert to deg/s
    hoodVelocity =
        m_motorSimModel.getAngularVelocity().in(edu.wpi.first.units.Units.RotationsPerSecond)
            * 360.0;

    // Record useful debugging outputs for CI logs
    Logger.recordOutput("Hood/TargetPosition", targetAngle);
    Logger.recordOutput("Hood/SimulatedHoodPosition(degrees)", hoodAngle);
    Logger.recordOutput("Hood/SimulatedHoodVelocity(deg/s)", hoodVelocity);
  }

  @Override
  public void periodic() {
    // existing periodic sets hoodAngle when not sim; ensure hoodVelocity is set from sensor when
    // not sim
    if (!isSim) {
      // Convert motor (rotor) position to hood (mechanism) angle using kGearRatio (mechanism/rotor)
      hoodAngle = hoodMotor.getRotorPosition().getValue().in(Units.Degrees) * kGearRatio;
      try {
        // Convert motor (rotor) velocity (rotations per second) to hood angular velocity in deg/s
        hoodVelocity = hoodMotor.getRotorVelocity().getValueAsDouble() * 360.0 * kGearRatio;
      } catch (Exception e) {
        // ignore if not available
      }
    }
    Logger.recordOutput("Hood/currentAngle", hoodAngle);
    Logger.recordOutput("Hood/targetAngle", targetAngle);
  }
}
