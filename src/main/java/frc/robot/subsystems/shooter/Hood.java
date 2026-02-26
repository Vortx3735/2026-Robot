// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private static final double kGearRatio = 1.0;
  private static final double kMOI = 0.001; // kg*m^2

  private final TalonFX hoodMotor;
  private final CANcoder canCoder;

  final DoubleEntry hoodMotorSpeedEntry;
  private double speed;

  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), kMOI, kGearRatio),
          DCMotor.getKrakenX44(1));

  public double hoodAngle = 0;
  public double targetAngle = 0;
  private final double error = 0.005;

  public Hood(int motorId, int canCoderId, Mode state) {
    hoodMotor = new TalonFX(motorId);
    canCoder = new CANcoder(canCoderId);
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
    slot0Configs.kD = 0.4; // Add 0.4 V output for a velocity error of 1 rps

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
    NetworkTable intakeTable = inst.getTable("Hood");
    hoodMotorSpeedEntry = intakeTable.getDoubleTopic("hoodMotorSpeed").getEntry(0);
    hoodMotorSpeedEntry.set(0.1);

    if (state == Mode.SIM) {
      var talonFXSim = hoodMotor.getSimState();
      talonFXSim.Orientation = ChassisReference.CounterClockwise_Positive;
      talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public double getHoodAngleDegrees() {
    return hoodMotor.getRotorPosition().getValueAsDouble()
        * 360.0
        / kGearRatio; // Convert to degrees
  }

  private void stop() {
    hoodMotor.set(0);
  }

  public void set(boolean reversed) {
    if (reversed) {
      hoodMotor.set(-speed);
    } else {
      hoodMotor.set(speed);
    }
  }

  public void setPositionPID(double degrees) {
    // final MotionMagicVoltage m_request = new MotionMagicVoltage((degrees / 360) * kGearRatio);
    final PositionVoltage m_request = new PositionVoltage((degrees / 360) * kGearRatio);
    hoodMotor.setControl(m_request);
    targetAngle = degrees;
  }

  public boolean isFinished() {
    double tolerance = 3;
    return Math.abs(targetAngle - hoodAngle) < tolerance;
  }

  public Command moveCommand(boolean reversed) {
    return this.run(
            () -> {
              setSpeed(hoodMotorSpeedEntry.getAsDouble());
              this.set(reversed);
            })
        .withName("move hood");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop hood");
  }

  public Command setPositionPIDCommand(double degrees) {
    return run(() -> setPositionPID(degrees)).withName("Set Hood Position PID");
  }

  public Command hold() {
    return run(() -> setPositionPID(targetAngle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hoodAngle = hoodMotor.getRotorPosition().getValue().in(Units.Degrees);
    Logger.recordOutput("Hood/currentAngle", hoodAngle);
    Logger.recordOutput("Hood/targetAngle", targetAngle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    var talonFXSim = hoodMotor.getSimState();
    var canCoderSim = canCoder.getSimState();

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

    // apply stuff to CANCoder
    canCoderSim.setRawPosition(m_motorSimModel.getAngularPosition());
    canCoderSim.setVelocity(m_motorSimModel.getAngularVelocity());

    hoodAngle = m_motorSimModel.getAngularPosition().in(Units.Degrees);
    Logger.recordOutput("Hood/TargetPosition", targetAngle);
    Logger.recordOutput("Hood/SimulatedHoodPosition(degrees)", hoodAngle);
  }
}
