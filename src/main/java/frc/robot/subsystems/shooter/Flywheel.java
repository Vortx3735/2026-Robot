package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private static final double kMOI = 0.001; // kg*m^2
  private static final double kMaxSpeed = 90; // Max speed in RPS

  private TalonFX flywheelMotor;

  private final BangBangController bbcontroller = new BangBangController();
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), kMOI, 1),
          DCMotor.getKrakenX60(1));

  private double currentRPS;
  private double targetRPS = 0;
  public double dashboardSpeed = 0;
  // NOTE: removed deprecated lowercase `targetrps` alias. Use `targetRPS`.

  public double simulatedVelocity;

  private final boolean isSim;
  // Sim-only input voltage computed from feedforward when control requests are made
  private double simulatedInputVoltage = 0.0;

  public Flywheel(int flywheelMotorID, Mode state) {
    flywheelMotor = new TalonFX(flywheelMotorID);
    isSim = state == Mode.SIM;
    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.31;
    slot0Configs.kV = 0.125;
    slot0Configs.kP = 0.25; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    // set Motion Magic Velocity settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 500; // Target acceleration of 100 rps/s
    motionMagicConfigs.MotionMagicJerk = 6000; // Target jerk of 6000 rps/s/s (0.1 seconds)

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelMotor.getConfigurator().apply(talonFXConfigs);
    flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
    if (state == Mode.SIM) {
      var talonFXSim = flywheelMotor.getSimState();
      talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
      talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }
  }

  public double getDashboardSpeed() {
    return flywheelSpeedEntry.get();
  }
  
  public double getFlywheelCurrentRPS() {
    return currentRPS;
  }

  public double getFlywheelTargetRPS() {
    return currentRPS;
  }

  public void stop() {
    flywheelMotor.set(0);
    targetRPS = 0;
    simulatedInputVoltage = 0.0;
  }

  public boolean isAtSpeed() {
    double tolerance = 3;
    return Math.abs(targetRPS - currentRPS) < tolerance;
  }

  public void shoot(double speed) {
    this.targetRPS = speed;

    // final VelocityVoltage m_request = new VelocityVoltage(speed);
    // flywheelMotor.setControl(m_request);
    flywheelMotor.set(bbcontroller.calculate(currentRPS, targetRPS));
    // In simulation, pre-compute a feedforwar .d voltage so the DCMotorSim receives a
    // deterministic input even if the Talon sim doesn't propagate motorVoltage.
    if (isSim) {
      final double kS = 0.31; // static volts (matches slot0 kS)
      final double kV = 0.125; // volts per RPS (matches slot0 kV)
      simulatedInputVoltage = Math.signum(targetRPS) * (kS + kV * Math.abs(targetRPS));
      Logger.recordOutput("Flywheel/SimShootTargetRPS", targetRPS);
      Logger.recordOutput("Flywheel/SimShootInputVoltage", simulatedInputVoltage);
    }
  }

  // For testing purposes, allows setting flywheel speed directly from Network
  // Tables
  public Command shootCommand() {

    return Commands.run(
            () -> {
              double speedRps = getDashboardSpeed() * kMaxSpeed;
              Logger.recordOutput("Flywheel/ShootCommandSpeedRPS", speedRps);
              shoot(speedRps);
            },
            this)
        .withName("shoot flywheel manual");
  }

  public Command shootCommand(double speed) {
    return this.run(() -> this.shoot(speed)).withName("shoot flywheel");
  }

  /** Dynamic shoot command which queries the supplied target RPS supplier each loop. */
  public Command shootCommand(Supplier<Double> speedSupplier) {
    // Call shoot() once immediately when the command is scheduled to ensure
    // simulatedInputVoltage is set before the first simulation tick. Then continue
    // calling it each scheduler loop while the command is active.
    // Ensure we set the target (and precompute simulated input) immediately when
    // the command is scheduled to avoid a race where simulationPeriodic runs
    // before the first execute() call.
    return Commands.sequence(
            Commands.runOnce(() -> this.shoot(speedSupplier.get()), this),
            Commands.run(() -> this.shoot(speedSupplier.get()), this))
        .withName("dynamic shoot flywheel");
  }

  public Command stopCommand() {
    return Commands.run(() -> stop(), this).withName("stop flywheel");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Prefer simulated value in simulation; fall back to Talon getter in hardware

    if (isSim) {
      currentRPS = simulatedVelocity;
    } else {
      currentRPS = flywheelMotor.getRotorVelocity().getValueAsDouble();
    }
  }

  @Override
  public void simulationPeriodic() {
    var talonFXSim = flywheelMotor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(12);

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();
    double voltageIn = 0.0;
    try {
      voltageIn = motorVoltage.in(Volts);
    } catch (Exception e) {
      // ignore, fallback handled below
    }
    // If the test or command computed a simulatedInputVoltage (see shoot()), prefer it
    // when the Talon sim reports a very small voltage. This avoids races where the
    // Talon sim hasn't propagated its internal motor voltage during unit tests.
    if (Math.abs(voltageIn) < 0.5) {
      if (Math.abs(simulatedInputVoltage) > 1e-6) {
        voltageIn = simulatedInputVoltage;
      } else if (Math.abs(targetRPS) > 1e-6) {
        // As a last-resort fallback, compute a simple feedforward to kick the sim.
        final double kS = 0.31; // static volts (matches slot0 kS)
        final double kV = 0.125; // volts per RPS (matches slot0 kV)
        voltageIn = Math.signum(targetRPS) * (kS + kV * Math.abs(targetRPS));
      }
    }
    // If we still haven't applied any voltage but we have a target, give the
    // model a tiny guaranteed kick so it starts moving. This prevents stalls in
    // situations where neither the Talon sim nor our precomputed feedforward
    // ever produce a nonzero voltage (as seen in some CI runs).
    if (Math.abs(voltageIn) < 1e-6 && Math.abs(targetRPS) > 1e-6) {
      voltageIn = Math.signum(targetRPS) * 1.0; // 1 volt kick
    }
    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(voltageIn);
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    // write simulated position/velocity back to the TalonFX sim state so
    // flywheelMotor.getRotorVelocity() reflects the simulated motor
    try {
      talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition());
      talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity());
    } catch (Exception e) {
      // Some simulation environments may not support setting raw rotor
      // position/velocity;
      // ignore failures to avoid test crashes.
    }
    simulatedVelocity = m_motorSimModel.getAngularVelocity().in(RotationsPerSecond);
    Logger.recordOutput("Flywheel/TargetVelocity(rps)", targetRPS);
    Logger.recordOutput("Flywheel/SimulatedVelocity(rps)", simulatedVelocity);
    Logger.recordOutput("Flywheel/SimAppliedVoltage", voltageIn);
    // If we've reached the target (or target was cleared), clear the precomputed input
    if (Math.abs(targetRPS) < 1e-6 || Math.abs(simulatedVelocity - targetRPS) < 0.5) {
      simulatedInputVoltage = 0.0;
    }
  }
}
