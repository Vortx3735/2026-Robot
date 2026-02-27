package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ShooterCommandsTest {

  private static final double METERS_TO_FEET = 3.28084;

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  void testHorizontalDistance() {
    Pose2d a = new Pose2d(0, 0, new Rotation2d());
    Pose2d b = new Pose2d(3, 4, new Rotation2d());
    // Poses are in meters; getHorizontalDistanceToHub converts to feet (5m * 3.28084 ft/m)
    assertEquals(5.0 * METERS_TO_FEET, ShooterCommands.getHorizontalDistanceToHub(a, b), 1e-3);
  }

  @Test
  void testAngleRelative() {
    Pose2d robot = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d hub = new Pose2d(1, 0, new Rotation2d());
    double angle = ShooterCommands.getAngleRelativeToHub(robot, hub);
    assertEquals(0.0, angle, 1e-6);
  }

  @Test
  void testCalculateShooterRpsImpossible() {
    // pick an impossible geometry (short range and flat)
    double rps = ShooterCommands.calculateShooterRPS(0.1, 10.0);
    assertEquals(0.0, rps);
  }
}
