package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import java.util.function.Supplier;

public class ShooterCommands {
  // Cache Pose2d instances for hubs (avoid allocating in tight loops)
  private static final Pose2d RED_HUB_POSE2D =
      new Pose2d(
          Constants.FieldConstants.RED_HUB_POSE3D.getX(),
          Constants.FieldConstants.RED_HUB_POSE3D.getY(),
          new Rotation2d());

  private static final Pose2d BLUE_HUB_POSE2D =
      new Pose2d(
          Constants.FieldConstants.BLUE_HUB_POSE3D.getX(),
          Constants.FieldConstants.BLUE_HUB_POSE3D.getY(),
          new Rotation2d());
  // Cache left/right hub poses for side aiming
  private static final Pose2d RED_LEFT_POSE2D =
      new Pose2d(
          Constants.FieldConstants.RED_LEFT.getX(),
          Constants.FieldConstants.RED_LEFT.getY(),
          new Rotation2d());
  private static final Pose2d RED_RIGHT_POSE2D =
      new Pose2d(
          Constants.FieldConstants.RED_RIGHT.getX(),
          Constants.FieldConstants.RED_RIGHT.getY(),
          new Rotation2d());
  private static final Pose2d BLUE_LEFT_POSE2D =
      new Pose2d(
          Constants.FieldConstants.BLUE_LEFT.getX(),
          Constants.FieldConstants.BLUE_LEFT.getY(),
          new Rotation2d());
  private static final Pose2d BLUE_RIGHT_POSE2D =
      new Pose2d(
          Constants.FieldConstants.BLUE_RIGHT.getX(),
          Constants.FieldConstants.BLUE_RIGHT.getY(),
          new Rotation2d());

  /**
   * Calculates required flywheel RPS for a given hood angle.
   *
   * @param xs Horizontal distance to target in feet
   * @param thetaDegrees Hood angle in degrees (e.g. 65.0)
   * @return Required flywheel speed in Rotations Per Second (RPS)
   */
  public static double calculateShooterRPS(double xs, double thetaDegrees) {
    // 1. Physical Constants
    double g = 32.2; // Gravity (ft/s^2)
    double thetaRad = Math.toRadians(thetaDegrees);
    double h = 4.0; // Target Height (6) - Launch Height (2)
    double wheelDiameter = 4.0 / 12.0; // 4 inch wheel converted to feet

    // 2. Calculate Required Ball Exit Velocity (v)
    double cosTheta = Math.cos(thetaRad);
    double tanTheta = Math.tan(thetaRad);

    double numerator = g * Math.pow(xs, 2);
    double denominator = 2 * Math.pow(cosTheta, 2) * (xs * tanTheta - h);

    if (denominator <= 0) return 0.0; // Distance/angle combination is physically impossible

    double vBall = Math.sqrt(numerator / denominator); // Linear ft/s

    // 3. Convert Ball Velocity to Wheel RPS
    // For a single-wheel + hood: Wheel Surface Speed = 2 * Ball Velocity
    double wheelSurfaceSpeed = vBall * 2.0;
    double wheelCircumference = Math.PI * wheelDiameter;

    double rps = wheelSurfaceSpeed / wheelCircumference;

    // 4. Recovery/Efficiency Factor (Adjust based on testing)
    // Most FRC shooters lose ~10-15% to slip/compression
    double efficiencyFactor = 1.15;

    return rps * efficiencyFactor;
  }

  /** Conversion factor from meters to feet. */
  private static final double METERS_TO_FEET = 3.28084;

  /**
   * Returns horizontal distance to the hub (xs) in feet given robot and hub poses (in meters).
   * Extracted so multiple commands can reuse the same calculation in parallel.
   */
  public static double getHorizontalDistanceToHub(Pose2d robotPose, Pose2d hubPose) {
    // Pose2d field coordinates are in meters; convert to feet for calculateShooterRPS
    double dx = hubPose.getX() - robotPose.getX();
    double dy = hubPose.getY() - robotPose.getY();
    return Math.hypot(dx, dy) * METERS_TO_FEET;
  }

  /** Returns the angle from the robot's heading to the hub, normalized to [-pi, pi]. */
  public static double getAngleRelativeToHub(Pose2d robotPose, Pose2d hubPose) {
    double angleToHub =
        Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX());
    double robotYaw = robotPose.getRotation().getRadians();
    double angleRelative = angleToHub - robotYaw;
    return Math.atan2(Math.sin(angleRelative), Math.cos(angleRelative));
  }

  /** Return the hub pose for the current alliance (red or blue). */
  public static Pose2d getAllianceHubPose() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red
        ? RED_HUB_POSE2D
        : BLUE_HUB_POSE2D;
  }

  /** Choose the left/right hub based on robot Y (used by AimToSide). */
  public static Pose2d chooseSideHubPose(Pose2d robotPose) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      return robotPose.getY() > 4.05 ? RED_LEFT_POSE2D : RED_RIGHT_POSE2D;
    } else {
      return robotPose.getY() > 4.05 ? BLUE_RIGHT_POSE2D : BLUE_LEFT_POSE2D;
    }
  }

  /**
   * Returns a Command that continuously aims the turret at the hub using the provided pose
   * suppliers.
   */
  private static Command turretAimCommand(
      Turret turret, Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> hubPoseSupplier) {
    return Commands.run(
            () -> {
              Pose2d rp = robotPoseSupplier.get();
              Pose2d hp = hubPoseSupplier.get();
              double angleRelative = getAngleRelativeToHub(rp, hp);
              double rotations = angleRelative / (2 * Math.PI);
              turret.setPositionPID(rotations);
            },
            turret)
        .withName("turret aim");
  }

  public static Command AimToHub(
      Turret turret, Flywheel flywheel, Hood hood, Supplier<Pose2d> poseSupplier, double theta) {
    // create a supplier that computes target RPS from the live robot pose
    Supplier<Double> targetRpsSupplier =
        () -> {
          Pose2d rp = poseSupplier.get();
          Pose2d hp = getAllianceHubPose();
          double liveXs = getHorizontalDistanceToHub(rp, hp);
          return calculateShooterRPS(liveXs, theta);
        };

    Supplier<Pose2d> hubPoseSupplier = ShooterCommands::getAllianceHubPose;

    // While shooting, continuously aim the turret and hold the hood at the desired angle.
    // The flywheel shoot command acts as the deadline: when it finishes, aiming/hood are
    // interrupted.
    return Commands.deadline(
            flywheel.shootCommand(targetRpsSupplier),
            Commands.run(
                () -> {
                  Pose2d rp = poseSupplier.get();
                  Pose2d hp = hubPoseSupplier.get();
                  double angleRelative = getAngleRelativeToHub(rp, hp);
                  double rotations = angleRelative / (2 * Math.PI);
                  turret.setPositionPID(rotations);
                },
                turret),
            hood.setPositionPIDCommand(theta))
        .withName("AimToHub");
  }

  public static Command AimToSide(Turret turret, Supplier<Pose2d> poseSupplier) {
    Supplier<Pose2d> hubPoseSupplier = () -> chooseSideHubPose(poseSupplier.get());
    return turretAimCommand(turret, poseSupplier, hubPoseSupplier);
  }

  /**
   * Backwards-compatible overload: aim turret to hub using only turret and a pose supplier. This
   * does not control the flywheel or hood.
   */
  public static Command AimToHub(Turret turret, Supplier<Pose2d> poseSupplier) {
    Supplier<Pose2d> hubPoseSupplier = ShooterCommands::getAllianceHubPose;
    return turretAimCommand(turret, poseSupplier, hubPoseSupplier).withName("turretAimToHub");
  }
}
