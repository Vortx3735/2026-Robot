//Have Aaron check this
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//My imports so far, apparently build.gradle hasn't been added yet
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//Need to add constants? (saw import option)

public class Shooter extends SubsystemBase {
    public static TalonFX flywheelShooterMotor;
  
  public Shooter(int flywheelMotorID) {
    flywheelShooterMotor = new TalonFX(flywheelMotorID);

    /*Don't know if we need this yet, just put in cause saw in 2025 elevator code

    configureTalonFX();

    */
  }

  /*This is also part of the stolen code that I have no clue about

  public void configureTalonFX() {

  }

  */

  /*Also don't know if I need, saved for later use

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  */
}
