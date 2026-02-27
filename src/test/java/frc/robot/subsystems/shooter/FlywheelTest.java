package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class FlywheelTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Flywheel flywheel = new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    assertNotNull(flywheel, "Flywheel should be instantiated successfully");
  }

  @Test
  public void testInitialTargetRps() {
    Flywheel flywheel = new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    assertTrue(flywheel.targetRPS == 0.0, "Initial targetRPS should be 0");
  }

  @Test
  public void testIsAtSpeedInitially() {
    Flywheel flywheel = new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    // Both currentrps and targetrps start at 0, so isAtSpeed should return true
    assertTrue(flywheel.isAtSpeed(), "Flywheel should be at speed when both velocities are 0");
  }

  @Test
  public void testStopResetsTargetRps() {
    Flywheel flywheel = new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    flywheel.targetRPS = 50.0;
    flywheel.stop();
    assertTrue(flywheel.targetRPS == 0.0, "targetRPS should be 0 after stop()");
  }

  @Test
  public void testCommandsNotNull() {
    Flywheel flywheel = new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    assertNotNull(flywheel.shootCommand(), "setVelocityPIDCommand should not be null");
    assertNotNull(flywheel.stopCommand(), "stopCommand should not be null");
    assertNotNull(flywheel.shootCommand(), "shootCommand should not be null");
  }

  @Test
  public void testSimulatedVelocityRampsToTarget() {
    Flywheel flywheel = new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);

    double targetRPS = 40.0; // reasonable test target
    // Rather than relying on the command scheduler, directly call shoot()
    // to guarantee the simulatedInputVoltage and targetRPS are set before we
    // start ticking the simulation. The original implementation used a
    // dynamic command which was not reliably executed in the headless test
    // environment, causing the voltage to remain at 0 and the sim to stall.
    flywheel.shoot(targetRPS);

    var scheduler = edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance();
    scheduler.cancelAll();

    boolean reached = false;
    for (int i = 0; i < 1000; i++) {
      scheduler.run();
      flywheel.simulationPeriodic();
      flywheel.periodic();

      double sim = flywheel.simulatedVelocity;
      if (Math.abs(sim - targetRPS) < 2.5) {
        reached = true;
        break;
      }
    }

    assertTrue(reached, "Flywheel simulated velocity should ramp to near target RPS");
  }
}
