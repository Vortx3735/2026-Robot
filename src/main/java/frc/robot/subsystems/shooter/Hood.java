// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
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

public class Hood extends SubsystemBase {
  /*define objects and variables here (e.g. motors, sensors, variables)
   *for example:
  public final Motor motor1;
   */
  private final TalonFX hoodMotor;
  private final CANcoder canCoder;
  private static final double kGearRatio = 10.0;
  private static final double kMOI = 0.001; // kg*m^2
  public double simHoodRotation;
  public double targetRotations = 0;
  private double commandedPercent = 0;
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), kMOI, kGearRatio),
          DCMotor.getKrakenX44(1)); // honiel said it uses 44s

  /*initialize subsystem objects in constructor
   *for good practice, pass in any constants through the constructor
   */
  public Hood(int hoodMotorID, int canCoderID, Mode state) {
    hoodMotor = new TalonFX(hoodMotorID);
    canCoder = new CANcoder(canCoderID);

    // Config PID
    var slot0Configs = new Slot0Configs();

    slot0Configs.kS = 0.1; // Output to overcome static friction
    slot0Configs.kV = 0.12; // Velocity target
    slot0Configs.kP = 0.11; // Proportional
    slot0Configs.kI = 0; // Integral
    slot0Configs.kD = 0; // Derivative

    // Apply PID config to motor
    hoodMotor.getConfigurator().apply(slot0Configs);

    // Config motor sim state if mode is sim
    if (state == Mode.SIM) {
      var talonFXSim = hoodMotor.getSimState();
      talonFXSim.Orientation = ChassisReference.Clockwise_Positive;
      talonFXSim.setMotorType(TalonFXSimState.MotorType.KrakenX44);
    }
  }

  private void move(double speed) {
    commandedPercent = speed;
    hoodMotor.set(speed);
  }

  private void stop() {
    commandedPercent = 0;
    hoodMotor.set(0);
  }

  public void setPositionPID(double rotations) {
    // create a Motion Magic request, voltage output
    // if (Math.abs(turretPosition - rotations) > error) {
    // final MotionMagicVoltage m_request = new MotionMagicVoltage(rotations * kGearRatio);
    final PositionVoltage m_request = new PositionVoltage(rotations * kGearRatio);
    hoodMotor.setControl(m_request);
    // }
    targetRotations = rotations;
  }

  public Command setPositionPIDCommand(double rotations) {
    return run(() -> setPositionPID(rotations)).withName("Set Hood Position PID");
  }

  public Command moveCommand(double speed) {
    return run(() -> move(speed)).withName("move hood");
  }

  public Command stopCommand() {
    return run(() -> stop()).withName("stop hood");
  }

  public double getHoodAngleDegrees() {
    return canCoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    var talonFXSim = hoodMotor.getSimState();

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

    double rotation = m_motorSimModel.getAngularPosition().in(Units.Rotations);
    if (rotation <= 0.5 && rotation >= -0.5) simHoodRotation = rotation;
    Logger.recordOutput("Hood/speed", commandedPercent);
    Logger.recordOutput("Hood/targetRotations", targetRotations);
    Logger.recordOutput(
        "Hood/simulatedHoodAngle", m_motorSimModel.getAngularPosition().in(Units.Degrees));
  }
}
