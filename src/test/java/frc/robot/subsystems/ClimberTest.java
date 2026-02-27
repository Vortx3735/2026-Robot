package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ClimberTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Climber climber =
        new Climber(
            Constants.ClimberConstants.CLIMBER_MOTOR_ID_LEFT,
            Constants.ClimberConstants.CLIMBER_MOTOR_ID_RIGHT);
    assertNotNull(climber, "Climber should be instantiated successfully");
  }

  @Test
  public void testSetSpeed() {
    Climber climber =
        new Climber(
            Constants.ClimberConstants.CLIMBER_MOTOR_ID_LEFT,
            Constants.ClimberConstants.CLIMBER_MOTOR_ID_RIGHT);
    climber.setSpeed(0.5);
    assertTrue(climber.getSpeed() > 0, "Speed should be positive after setSpeed");
  }

  @Test
  public void testCommandsNotNull() {
    Climber climber =
        new Climber(
            Constants.ClimberConstants.CLIMBER_MOTOR_ID_LEFT,
            Constants.ClimberConstants.CLIMBER_MOTOR_ID_RIGHT);
    assertNotNull(climber.upCommand(), "upCommand should not be null");
    assertNotNull(climber.downCommand(), "downCommand should not be null");
    assertNotNull(climber.stopCommand(), "stopCommand should not be null");
  }
}
