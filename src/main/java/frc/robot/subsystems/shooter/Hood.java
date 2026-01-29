// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  public double simHoodAngle;
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), kMOI, kGearRatio),
          DCMotor.getKrakenX44(1)); // honiel said it uses 44s

  /*initialize subsystem objects in constructor
   *for good practice, pass in any constants through the constructor
   */
  public Hood(int hoodMotorID, int canCoderID) {
    hoodMotor = new TalonFX(hoodMotorID);
    canCoder = new CANcoder(canCoderID);
  }

  public void run(double speed) {
    hoodMotor.set(speed);
  }

  public void stop() {
    hoodMotor.set(0);
  }

  public Command runCommand(double speed) {
    return new RunCommand(() -> run(speed), this).withName("run hood");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop hood");
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
    m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
    talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));

    simHoodAngle = m_motorSimModel.getAngularPosition().in(Units.Rotations);
    Logger.recordOutput("Hood/SimulatedHoodAngle", simHoodAngle);
  }
}
