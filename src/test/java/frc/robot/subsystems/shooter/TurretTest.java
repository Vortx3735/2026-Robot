package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class TurretTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Turret turret = new Turret(Constants.TurretConstants.TURRET_MOTOR_ID, Mode.SIM);
    assertNotNull(turret, "Turret should be instantiated successfully");
  }

  @Test
  public void testInitialTargetRotations() {
    Turret turret = new Turret(Constants.TurretConstants.TURRET_MOTOR_ID, Mode.SIM);
    assertEquals(0.0, turret.targetPosition, 1e-6, "Initial targetPosition should be 0");
  }

  @Test
  public void testInitialTurretPosition() {
    Turret turret = new Turret(Constants.TurretConstants.TURRET_MOTOR_ID, Mode.SIM);
    assertEquals(0.0, turret.getCurrentPosition(), 1e-6, "Initial turret position should be 0");
  }

  @Test
  public void testSetSpeed() {
    Turret turret = new Turret(Constants.TurretConstants.TURRET_MOTOR_ID, Mode.SIM);
    // Turret does not expose a public speed field; ensure moveCommand exists
    assertNotNull(turret.moveCommand(false), "moveCommand(false) should not be null");
  }

  @Test
  public void testSetPositionPIDUpdatesTargetRotations() {
    Turret turret = new Turret(Constants.TurretConstants.TURRET_MOTOR_ID, Mode.SIM);
    turret.setPositionPID(0.25);
    assertEquals(0.25, turret.targetPosition, 1e-6, "targetPosition should be set to 0.25");
  }

  @Test
  public void testCommandsNotNull() {
    Turret turret = new Turret(Constants.TurretConstants.TURRET_MOTOR_ID, Mode.SIM);
    assertNotNull(turret.moveCommand(false), "moveCommand(false) should not be null");
    assertNotNull(turret.moveCommand(true), "moveCommand(true) should not be null");
    assertNotNull(turret.stopCommand(), "stopCommand should not be null");
    assertNotNull(turret.setPositionPIDCommand(0.0), "setPositionPIDCommand should not be null");
    assertNotNull(
        turret.setPositionPIDCommandManualSetpoint(),
        "setPositionPIDCommandManualSetpoint should not be null");
  }

  @Test
  public void testSetPositionCommandFinishes() {
    Turret turret = new Turret(Constants.TurretConstants.TURRET_MOTOR_ID, Mode.SIM);
    var cmd = turret.setPositionPIDCommand(0.5);
    var scheduler = edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.schedule(cmd);

    boolean finished = false;
    for (int i = 0; i < 400; i++) {
      scheduler.run();
      turret.simulationPeriodic();
      turret.periodic();
      if (cmd.isFinished()) {
        finished = true;
        break;
      }
    }
    assertTrue(finished, "Turret setPositionPIDCommand should finish");
  }
}
