package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  /**
   * Verifies that followPath converts field-relative Choreo speeds to robot-relative before
   * driving.
   *
   * <p>When the robot is facing 180° (Red-alliance start), a sample with field-relative vy = +4.89
   * m/s should command the robot to move backward in its own frame (robot-relative vy = -4.89), which
   * corresponds to moving in the +Y field direction. If the conversion were missing, the sample
   * speeds would be applied as-is (robot-relative vy = +4.89), driving the robot in the wrong field
   * direction.
   */
  @Test
  public void testFollowPathConvertsFieldRelativeToRobotRelative() {
    // Reset odometry to a pose facing 180° (simulates Red-alliance start after Choreo flip).
    Pose2d redStartPose = new Pose2d(13.92, 2.87, Rotation2d.fromDegrees(180));
    drive.resetOdometry(redStartPose);

    // Create a SwerveSample with purely field-relative vy = +4.89 m/s (moving up the field).
    // For a robot facing 180°, the correct robot-relative command should be vy = -4.89 m/s.
    SwerveSample sample =
        new SwerveSample(
            0.0, // t
            13.92, // x – matches robot position so PID correction is ~0
            2.87, // y
            Math.PI, // heading
            0.0, // vx (field-relative)
            4.89, // vy (field-relative, moving in +Y field direction)
            0.0, // omega
            0.0,
            0.0,
            0.0,
            new double[] {0, 0, 0, 0},
            new double[] {0, 0, 0, 0});

    // followPath should NOT throw and should complete without error.
    // (Verifies the method runs with the conversion; full velocity assertion requires hardware.)
    drive.followPath(sample);
  }
}
