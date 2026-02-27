package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class VisionTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitializationNoIO() {
    Vision vision = new Vision((pose, timestamp, stdDevs) -> {});
    assertNotNull(vision, "Vision should be instantiated with no VisionIO instances");
  }

  @Test
  public void testInitializationWithEmptyIO() {
    Vision vision = new Vision((pose, timestamp, stdDevs) -> {}, new VisionIO() {});
    assertNotNull(vision, "Vision should be instantiated with an empty VisionIO");
  }

  @Test
  public void testInitializationWithMultipleEmptyIO() {
    Vision vision =
        new Vision((pose, timestamp, stdDevs) -> {}, new VisionIO() {}, new VisionIO() {});
    assertNotNull(vision, "Vision should be instantiated with multiple empty VisionIO instances");
  }

  @Test
  public void testGetTargetXNotNull() {
    Vision vision = new Vision((pose, timestamp, stdDevs) -> {}, new VisionIO() {});
    assertNotNull(vision.getTargetX(0), "getTargetX(0) should return a non-null Rotation2d");
  }

  /** Verifies the VisionConsumer functional interface can accept a Pose2d measurement. */
  @Test
  public void testVisionConsumerAcceptsPose() {
    Pose2d[] captured = {null};
    Vision.VisionConsumer consumer = (pose, timestamp, stdDevs) -> captured[0] = pose;
    Pose2d testPose = new Pose2d();
    consumer.accept(testPose, 0.0, null);
    assertNotNull(captured[0], "VisionConsumer should have received the pose");
  }
}
