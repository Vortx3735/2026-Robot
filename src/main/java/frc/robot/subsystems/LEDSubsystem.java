package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.WLEDController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  private BooleanSupplier idling;
  private LEDState color = LEDState.OFF;
  private int id = 3;

  public LEDSubsystem(BooleanSupplier idling) {
    this.idling = idling;
  }

  public enum LEDState {
    OFF(-1),
    IDLE(1),
    CHASE(2),
    RED(3),
    BLUE(5),
    GREEN(7),
    RED_BLINK(4),
    BLUE_BLINK(6),
    GREEN_BLINK(8),
    YELLOW(9),
    YELLOW_BLINK(10);

    private final int presetId;

    LEDState(int id) {
      this.presetId = id;
    }
  }

  public void setColor(LEDState color) {
    this.color = color;
  }

  public Command setColorCommand(LEDState color) {
    return this.run(() -> setColor(color));
  }

  private final WLEDController wled = new WLEDController(Constants.WLED_IP);
  private LEDState currentState = LEDState.OFF;

  public void setState(LEDState newState) {
    if (newState == currentState) return; // Optimization

    if (newState == LEDState.OFF) {
      wled.setPower(false);
    } else {
      wled.setPower(true);
      wled.setPreset(newState.presetId);
    }
    currentState = newState;
  }

  public LEDState getState() {
    return currentState;
  }

  public Command setStateCommand(LEDState state) {
    return run(() -> setState(state));
  }

  // public Command blinkCommand() {
  //   if (getState() == LEDState.BLUE) {
  //     return setStateCommand(LEDState.BLUE_BLINK);
  //   } else if (getState() == LEDState.RED) {
  //     return setStateCommand(LEDState.RED_BLINK);
  //   } else if(getState() == LEDState.GREEN){
  //     return setStateCommand(LEDState.GREEN_BLINK);
  //   } else{
  //     return setStateCommand(LEDState.YELLOW_BLINK);
  //   }
  // }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          this.setState(LEDState.RED);
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
          this.setState(LEDState.BLUE);
        } else {
          this.setState(LEDState.IDLE);
        }
      } else {
        this.setState(LEDState.IDLE);
      }
    } else {
      SmartDashboard.putBoolean("ledidling", idling.getAsBoolean());
      SmartDashboard.putNumber("ledid", id);
      if (idling.getAsBoolean()) {
        setState(color);
        // SmartDashboard.putBoolean("ledstate", false);

      } else {
        if (color == LEDState.RED) {
          setState(LEDState.RED_BLINK);
        } else if (color == LEDState.BLUE) {
          setState(LEDState.BLUE_BLINK);
        } else if (color == LEDState.GREEN) {
          setState(LEDState.GREEN_BLINK);
        } else if (color == LEDState.YELLOW) {
          setState(LEDState.YELLOW_BLINK);
        }
        SmartDashboard.putBoolean("ledstate", true);
      }
    }

    Logger.recordOutput("LED/currentState", getState());
  }
}
