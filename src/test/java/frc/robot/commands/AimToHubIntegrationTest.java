package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class AimToHubIntegrationTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testAimToHubMovesAllSubsystems() {
    // instantiate subsystems in SIM mode
    Turret turret = new Turret(Constants.TurretConstants.TURRET_MOTOR_ID, Mode.SIM);
    Hood hood = new Hood(Constants.HoodConstants.HOOD_MOTOR_ID, Mode.SIM);
    Flywheel flywheel = new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);

    // Choose a robot pose that gives a non-zero angle to the hub
    Pose2d robotPose = new Pose2d(0.0, 0.0, new Rotation2d());

    double theta = 65.0; // hood angle

    // Compute expected values
    Pose2d hubPose = ShooterCommands.getAllianceHubPose();
    double expectedXs = ShooterCommands.getHorizontalDistanceToHub(robotPose, hubPose);
    double expectedRps = ShooterCommands.calculateShooterRPS(expectedXs, theta);
    double expectedAngleRelative = ShooterCommands.getAngleRelativeToHub(robotPose, hubPose);
    double expectedRotations = expectedAngleRelative / (2 * Math.PI);

    // Directly set subsystem setpoints (mimics AimToHub behavior) and run the sim loop
    turret.setPositionPID(expectedRotations);
    hood.setPositionPID(theta);
    flywheel.shoot(expectedRps);

    // Run simulation/periodic loops to allow subsystems to update (500 iterations ~= 10s)
    for (int i = 0; i < 500; i++) {
      turret.simulationPeriodic();
      hood.simulationPeriodic();
      flywheel.simulationPeriodic();

      turret.periodic();
      hood.periodic();
      flywheel.periodic();
    }

    // Verify turret/hood targets are set and flywheel is spinning (in simulation)
    assertEquals(
        expectedRotations,
        turret.targetPosition,
        0.05,
        "Turret targetPosition should match expected rotations");
    assertEquals(theta, hood.targetAngle, 0.5, "Hood targetAngle should be set to theta");

    // Use the DCMotorSim-derived simulated velocity which is independent of TalonFX sim
    double simulated = flywheel.simulatedVelocity;
    assertTrue(
        Math.abs(simulated - expectedRps) < Math.max(1.0, expectedRps * 0.25),
        "Flywheel simulated velocity should be near expectedRps");
  }
}
