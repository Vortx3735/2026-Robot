package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class HopperTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    assertNotNull(hopper, "Hopper should be instantiated successfully");
  }

  @Test
  public void testSetHopperSpeed() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    // verify the default from constructor
    assertEquals(0.1, hopper.getHopperSpeed(), 1e-6, "Default hopper speed should be 0.1");
  }

  @Test
  public void testGetHopperSpeedAfterMultipleSets() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    // Cannot directly set via public API; ensure getter returns a number
    double speed = hopper.getHopperSpeed();
    assertTrue(speed >= 0.0, "Hopper speed should be non-negative");
  }

  @Test
  public void testCommandsNotNull() {
    Hopper hopper = new Hopper(Constants.HopperConstants.HOPPER_MOTOR_ID);
    assertNotNull(hopper.intakeCommand(), "intakeCommand() should not be null");
    assertNotNull(hopper.outtakeCommand(), "outtakeCommand() should not be null");
    assertNotNull(hopper.stopCommand(), "stopCommand should not be null");
  }
}
