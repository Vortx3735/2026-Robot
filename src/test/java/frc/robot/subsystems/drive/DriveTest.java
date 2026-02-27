package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class DriveTest {

  private static Drive drive;

  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            (robotPose) -> {});
  }

  @Test
  public void testInitialization() {
    assertNotNull(drive, "Drive should be instantiated successfully");
  }

  @Test
  public void testGetPoseNotNull() {
    assertNotNull(drive.getPose(), "getPose() should return a non-null Pose2d");
  }

  @Test
  public void testGetRotationNotNull() {
    assertNotNull(drive.getRotation(), "getRotation() should return a non-null Rotation2d");
  }

  @Test
  public void testGetMaxLinearSpeedPositive() {
    assertTrue(drive.getMaxLinearSpeedMetersPerSec() > 0, "Max linear speed should be positive");
  }

  @Test
  public void testGetMaxAngularSpeedPositive() {
    assertTrue(drive.getMaxAngularSpeedRadPerSec() > 0, "Max angular speed should be positive");
  }

  @Test
  public void testGetModuleTranslationsFour() {
    assertEquals(
        4, Drive.getModuleTranslations().length, "Drive should have 4 module translations");
  }

  @Test
  public void testResetOdometry() {
    Pose2d newPose = new Pose2d();
    drive.resetOdometry(newPose);
    assertNotNull(drive.getPose(), "Pose should not be null after odometry reset");
  }
}
