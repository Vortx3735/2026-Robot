package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class HoodTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Hood hood = new Hood(Constants.HoodConstants.HOOD_MOTOR_ID, Mode.SIM);
    assertNotNull(hood, "Hood should be instantiated successfully");
  }

  @Test
  public void testSetPositionPIDUpdatesTargetAngle() {
    Hood hood = new Hood(Constants.HoodConstants.HOOD_MOTOR_ID, Mode.SIM);
    hood.setPositionPID(45.0);
    assertEquals(45.0, hood.targetAngle, 1e-6, "targetAngle should be set to 45.0 degrees");
  }

  @Test
  public void testCommandsNotNull() {
    Hood hood = new Hood(Constants.HoodConstants.HOOD_MOTOR_ID, Mode.SIM);
    assertNotNull(hood.moveCommand(false), "moveCommand(false) should not be null");
    assertNotNull(hood.moveCommand(true), "moveCommand(true) should not be null");
    assertNotNull(hood.stopCommand(), "stopCommand should not be null");
    assertNotNull(hood.setPositionPIDCommand(0.0), "setPositionPIDCommand should not be null");
    assertNotNull(hood.hold(), "hold should not be null");
  }
}
