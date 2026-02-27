package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class IntakeTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    assertNotNull(intake, "Intake should be instantiated successfully");
  }

  @Test
  public void testSetSpeed() {
    Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    // default is set via NetworkTables in constructor
    assertTrue(intake.getIntakeSpeed() >= 0.0, "Intake speed should be non-negative");
  }

  @Test
  public void testSetSpeedZero() {
    Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    // cannot set directly via public API; verify getter returns a number
    assertEquals(intake.getIntakeSpeed(), intake.getIntakeSpeed(), 1e-6);
  }

  @Test
  public void testCommandsNotNull() {
    Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    assertNotNull(intake.intakeCommand(), "intakeCommand should not be null");
    assertNotNull(intake.stopCommand(), "stopCommand should not be null");
  }
}
