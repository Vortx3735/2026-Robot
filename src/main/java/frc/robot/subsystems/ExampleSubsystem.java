// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import stuff up here
// for example:
// import com.aaronsFavWebsite.motorLibrary.Motor;

public class ExampleSubsystem extends SubsystemBase {
  /*define objects and variables here (e.g. motors, sensors, variables)
   *for example:
   *public final Motor subsystemNameMotor;
   */

  /*initialize subsystem objects in constructor
   *for good practice, pass in any constants through the constructor
   */
  public ExampleSubsystem(/*take in can ids here  (for example:int subsystemNameMotorID)*/ ) {
    /*initialize motors from port numbers etc.
     *also send configurations to the motor
     *for example:
     *this.motor1 = new Motor(motorId);
     *motor1.reverse(true)
     */
  }

  public void exampleMethod() {
    /*example method for the subsystem
     *for example, set motor speed, read sensor value, etc.
     *example:
     *this.motor1.setSpeed(0.5);
     */
  }

  /*an example method that returns a very basic command
   *only create commands in the subystem if they only utilize methods from the same subystem
   */
  public Command exampleMethodCommand() {
    /*return an inline command
     *this example uses runOnce, but you can also use run or startEnd depending on your needs
     */
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
