// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.xml.transform.Source;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import stuff up here
// for example:
// import com.aaronsFavWebsite.motorLibrary.Motor;

public class Intake2 extends SubsystemBase {
  /*define objects and variables here (e.g. motors, sensors, variables)
   *for example:
   *public final Motor motor1;
   */

  /*initialize subsystem objects in constructor
   *for good practice, pass in any constants through the constructor
   */
  
  /* 
   public class Robot extends TimedRobot {
   private static final CANBus kCANBus = new CANBus("canivore");

   private final TalonFX m_leftLeader = new TalonFX(0);
   private final TalonFX m_rightLeader = new TalonFX(1);
   private final TalonFX m_leftFollower = new TalonFX(2);
   private final TalonFX m_rightFollower = new TalonFX(3);

   private final DutyCycleOut m_leftOut = new DutyCycleOut(0);
   private final DutyCycleOut m_rightOut = new DutyCycleOut(0);

   private final XboxController m_driverJoy = new XboxController(0);

   public Robot() {
      // start with factory-default configs
      var currentConfigs = new MotorOutputConfigs();

      // The left motor is CCW+
      currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
      m_leftLeader.getConfigurator().apply(currentConfigs);

      // The right motor is CW+
      currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
      m_rightLeader.getConfigurator().apply(currentConfigs);

      // Ensure our followers are following their respective leader
      m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
      m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));
   }

   @Override
   public void teleopPeriodic() {
      // retrieve joystick inputs
      var fwd = -m_driverJoy.getLeftY();
      var rot = m_driverJoy.getRightX();

      // modify control requests
      m_leftOut.Output = fwd + rot;
      m_rightOut.Output = fwd - rot;

      // send control requests
      m_leftLeader.setControl(m_leftOut);
      m_rightLeader.setControl(m_rightOut);
   }
}
  */

private double speed;
private final TalonFX intakeMotor;

  // Network Table Entry
  //final DoubleEntry intakeMotorSpeedEntry;

public Intake2(int motorId) {
    /*initialize motors from port numbers etc.
     *also send configurations to the motor
     *for example:
     *this.motor1 = new Motor(motorId);
     *motor1.reverse(true)
     */
    intakeMotor = new TalonFX(motorId);
    speed = 0.5;
  }

  public void setSpeed(double speed) {
        /*example method for the subsystem
         *for example, set motor speed, read sensor value, etc.
         *example:
         *this.motor1.setSpeed(0.5
         */
          this.speed = speed;
  }

   public void stopIntake() {
    // Stop motor
    intakeMotor.set(0);
  }

  public void intake() {
    intakeMotor.set(speed);
  }

  public Command intakeCommand() {
    return this.run(
            () -> {
              this.intake();
            })
        .withName("run intake");
  }

  public Command stopCommand() {
    return this.run(() -> {
      this.stopIntake();
    })
    .withName("stop intake");
  }


  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}

//C++ (Source)

//C++ (Header)



  /*an example method that returns a very basic command
   *only create commands in the subystem if they only utilize methods from the same subystem
   */
  /* 
   public Command exampleMethodCommand() {
    /*return an inline command
     *this example uses runOnce, but you can also use run or startEnd depending on your needs
     */
    /*
     return this.runOnce(
        () ->
            // single method goes here
            // for example:
            this.exampleMethod());
  }

  // an example getter for the motors, sensors,variables, etc. of the subsystem
  public boolean getSomeValue() {
    /*return some value from the subsystem
     *for example:
     *return this.motor.getSpeed()
     */
   /*  return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  */

