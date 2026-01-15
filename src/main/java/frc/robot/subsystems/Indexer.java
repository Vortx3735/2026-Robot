// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import stuff up here
// for example:
// import com.aaronsFavWebsite.motorLibrary.Motor;

public class Indexer extends SubsystemBase {
  /*define objects and variables here (e.g. motors, sensors, variables)
   *for example:
  public final Motor motor1;
   */
   private final TalonFX motor;

  

  /*initialize subsystem objects in constructor
   *for good practice, pass in any constants through the constructor
   */
  public Indexer(int motorId) {
    motor=new TalonFX(motorId);
    
  }

  public void setSpeed(double speed) {
    this.motor.set(speed);
    /*example method for the subsystem
     *for example, set motor speed, read sensor value, etc.
     *example:
     *this.motor1.setSpeed(0.);
     */
  }

  /*an example method that returns a very basic command
   *only create commands in the subystem if they only utilize methods from the same subystem
   */
  /*public Command exampleMethodCommand() {
    /*return an inline command
     *this example uses runOnce, but you can also use run or startEnd depending on your needs
     */
    /*return this.runOnce(
        () ->
            // single method goes here
            // for example:
            this.exampleMethod());
  }*/

  // an example getter for the motors, sensors,variables, etc. of the subsystem
  public boolean getSomeValue() {
    /*return some value from the subsystem
     *for example:
     *return this.motor.getSpeed()
     */
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
