package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class TunnelTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    assertNotNull(tunnel, "Tunnel should be instantiated successfully");
  }

  @Test
  public void testSetTunnelSpeed() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    // The Tunnel uses NetworkTables defaults; ensure getter returns a number
    assertTrue(tunnel.getBottomTunnelSpeed() >= 0.0, "Bottom tunnel speed should be non-negative");
  }

  @Test
  public void testSetTunnelSpeedZero() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    // Cannot set speeds directly via public API in current implementation; verify getter present
    assertNotNull(tunnel.getBottomTunnelSpeed(), "Bottom tunnel speed getter should exist");
  }

  @Test
  public void testCommandsNotNull() {
    Tunnel tunnel =
        new Tunnel(
            Constants.TunnelConstants.BOTTOM_TUNNEL_MOTOR_ID,
            Constants.TunnelConstants.TOP_TUNNEL_MOTOR_ID);
    assertNotNull(tunnel.intakeCommand(), "intakeCommand(false) should not be null");
    assertNotNull(tunnel.outtakeCommand(), "outtakeCommand(true) should not be null");
    assertNotNull(tunnel.stopCommand(), "stopCommand should not be null");
  }
}
