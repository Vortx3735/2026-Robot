package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Tunnel;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.util.ShotLUT;
import frc.robot.util.ShotParameters;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {
  private static final ShotLUT shotLUT = new ShotLUT();

  static {
    // format: (distance (meters) -> {RPM, timeOfFlight})
    // SHOOTER_MAP.put();

    // hood angle 65 deg
    shotLUT.put(0.85, new ShotParameters(1500, 0.322));
    shotLUT.put(0.90, new ShotParameters(1500, 0.341));
    shotLUT.put(2.05, new ShotParameters(1500, 0.791));
    shotLUT.put(2.10, new ShotParameters(1500, 0.811));
    shotLUT.put(2.15, new ShotParameters(1509, 0.826));
    shotLUT.put(2.20, new ShotParameters(1518, 0.841));
    shotLUT.put(2.25, new ShotParameters(1526, 0.856));
    shotLUT.put(2.30, new ShotParameters(1544, 0.865));
    shotLUT.put(2.35, new ShotParameters(1553, 0.880));
    shotLUT.put(2.40, new ShotParameters(1566, 0.892));
    shotLUT.put(2.45, new ShotParameters(1579, 0.904));
    shotLUT.put(2.50, new ShotParameters(1588, 0.918));
    shotLUT.put(2.55, new ShotParameters(1605, 0.927));
    shotLUT.put(2.60, new ShotParameters(1614, 0.941));
    shotLUT.put(2.65, new ShotParameters(1627, 0.952));
    shotLUT.put(2.70, new ShotParameters(1641, 0.963));
    shotLUT.put(2.75, new ShotParameters(1649, 0.976));
    shotLUT.put(2.80, new ShotParameters(1667, 0.985));
    shotLUT.put(2.85, new ShotParameters(1676, 0.998));
    shotLUT.put(2.90, new ShotParameters(1689, 1.008));
    shotLUT.put(2.95, new ShotParameters(1702, 1.019));
    shotLUT.put(3.00, new ShotParameters(1711, 1.032));
    shotLUT.put(3.05, new ShotParameters(1724, 1.042));
    shotLUT.put(3.10, new ShotParameters(1737, 1.052));
    shotLUT.put(3.15, new ShotParameters(1746, 1.064));
    shotLUT.put(3.20, new ShotParameters(1764, 1.071));
    shotLUT.put(3.25, new ShotParameters(1772, 1.084));
    shotLUT.put(3.30, new ShotParameters(1786, 1.093));
    shotLUT.put(3.35, new ShotParameters(1799, 1.103));
    shotLUT.put(3.40, new ShotParameters(1808, 1.115));
    shotLUT.put(3.45, new ShotParameters(1821, 1.124));
    shotLUT.put(3.50, new ShotParameters(1834, 1.133));
    shotLUT.put(3.55, new ShotParameters(1843, 1.145));
    shotLUT.put(3.60, new ShotParameters(1860, 1.151));
    shotLUT.put(3.65, new ShotParameters(1869, 1.163));
    shotLUT.put(3.70, new ShotParameters(1882, 1.172));
    shotLUT.put(3.75, new ShotParameters(1896, 1.180));
    shotLUT.put(3.80, new ShotParameters(1904, 1.192));
    shotLUT.put(3.85, new ShotParameters(1917, 1.200));
    shotLUT.put(3.90, new ShotParameters(1931, 1.209));
    shotLUT.put(3.95, new ShotParameters(1939, 1.220));
    shotLUT.put(4.00, new ShotParameters(1953, 1.228));
    shotLUT.put(4.05, new ShotParameters(1966, 1.236));
    shotLUT.put(4.10, new ShotParameters(1975, 1.247));
    shotLUT.put(4.15, new ShotParameters(1988, 1.255));
    shotLUT.put(4.20, new ShotParameters(2001, 1.263));
    shotLUT.put(4.25, new ShotParameters(2010, 1.273));
    shotLUT.put(4.30, new ShotParameters(2023, 1.281));
    shotLUT.put(4.35, new ShotParameters(2032, 1.292));
    shotLUT.put(4.40, new ShotParameters(2045, 1.299));
    shotLUT.put(4.45, new ShotParameters(2058, 1.307));
    shotLUT.put(4.50, new ShotParameters(2067, 1.317));
    shotLUT.put(4.55, new ShotParameters(2080, 1.325));
    shotLUT.put(4.60, new ShotParameters(2089, 1.335));
    shotLUT.put(4.65, new ShotParameters(2102, 1.342));
    shotLUT.put(4.70, new ShotParameters(2115, 1.349));
    shotLUT.put(4.75, new ShotParameters(2124, 1.359));
    shotLUT.put(4.80, new ShotParameters(2137, 1.367));
    shotLUT.put(4.85, new ShotParameters(2150, 1.374));
    shotLUT.put(4.90, new ShotParameters(2159, 1.383));
    shotLUT.put(4.95, new ShotParameters(2172, 1.390));
    shotLUT.put(5.00, new ShotParameters(2181, 1.400));
  }

  public static double latencyConstant =
      0.15; // time in s of latency (pose updating, rio delay, etc)
  // if only running right camera
  public static double offset = 0.09;
  public static double efficiencyFactor = 1.005;
  // public double offset=0.0;
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

  // Iterates through the LUT (distance -> velocity) to find an appropriate distance.
  public static double velocityToEffectiveDistance(double velocity) {
    for (Map.Entry<Double, ShotParameters> entry : shotLUT.entrySet()) {
      double dist = entry.getKey();
      double vel = dist / entry.getValue().tofSec();
      if (vel == velocity) {
        return dist;
      }
    }

    return shotLUT.lastKey(); // default/clamp distance is max distance measured/interpolated
  }

  public static Translation2d SOTMgetShotVelocity(
      Pose2d robotPosition, Translation2d robotVelocity) {
    // predict future robot postion
    Translation2d futurePos =
        robotPosition.getTranslation().plus(
            robotVelocity.times(latencyConstant));
    Logger.recordOutput("Shooter/futurePos", futurePos);

    // get the target vector (translation from robot to hub)
    Translation2d toHub = getAllianceHubPose().getTranslation().minus(futurePos).minus(new Translation2d(8.325 - 0.5969, 0));
    Logger.recordOutput("Shooter/xDisHub", toHub.getX());
    Logger.recordOutput("Shooter/yDisHub", toHub.getY());
    double distance = toHub.getNorm();
    // double distance = getDistanceToHub(futurePos, getAllianceHubPose()) / 3.281; // this also
    // works
    Translation2d targetDirection = toHub.div(distance);

    Logger.recordOutput("Shooter/distanceSOTM", distance);

    // calculate baseline required horizontal velocity
    ShotParameters params = shotLUT.get(distance);
    double horiVelocity = distance / params.tofSec();
    Logger.recordOutput("Shooter/baselineVelo", horiVelocity);

    // build target velocity vector
    Translation2d targetVelocity = targetDirection.times(horiVelocity);

    // subtract robot velocity vectoramabob
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    return shotVelocity;
  }

  public static Command SOTMShoot(
      Pose2d robotPosition,
      Translation2d robotVelocity,
      Tunnel tunnel,
      Hopper hopper,
      Flywheel flywheel,
      Intake intake) {

    Supplier<Double> targetRpsSupplier =
      () -> {
        Translation2d shotVelocity = SOTMgetShotVelocity(robotPosition, robotVelocity);

        // get our nice little results
        double requiredVelocity = shotVelocity.getNorm();
        Logger.recordOutput("Shooter/reqVelo", requiredVelocity);

        // look up the required distance for that velocity
        double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
        Logger.recordOutput("Shooter/SOTMEffectiveDistance", effectiveDistance);

        return shotLUT.get(effectiveDistance).rpm() / 60.0;
    };

    // look up the required rpm for that distance
    return CommandFactory.shootCommand(
            flywheel, tunnel, hopper, intake, targetRpsSupplier)
        .withName("SOTMShoot");
  }

  public static Command SOTMAim(
      Pose2d robotPosition, Translation2d robotVelocity, Turret turret, Hood hood) {
    Translation2d shotVelocity = SOTMgetShotVelocity(robotPosition, robotVelocity);

    // get our nice little results
    double turretRotation = shotVelocity.getAngle().getRotations();
    return Commands.parallel(
        hood.setPositionPIDCommand(65), turret.setPositionPIDCommand(turretRotation));
  }

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
    double h = 4.11776908; // Target Height (6) - Launch Height (1.88223092)
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
    double ef = efficiencyFactor;
    if (xs > 13) {
      ef += 0.04;
    }
    Logger.recordOutput("Shooter/calculatedShooterRPS", rps * ef);
    return rps * ef;
  }

  /** Conversion factor from meters to feet. */
  private static final double METERS_TO_FEET = 3.28084;

  /**
   * Returns horizontal distance to the hub (xs) in feet given robot and hub poses (in meters).
   * Extracted so multiple commands can reuse the same calculation in parallel.
   */
  public static double getDistanceToHub(Pose2d robotPose, Pose2d hubPose) {
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
    return -Math.atan2(Math.sin(angleRelative), Math.cos(angleRelative)) + offset;
  }

  /** Return the hub pose for the current alliance (red or blue). */
  public static Pose2d getAllianceHubPose() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red
        ? RED_HUB_POSE2D
        : BLUE_HUB_POSE2D;
  }

  /** Choose the left/right hub based on robot Y (used by AimToSide). */
  public static Pose2d getSidePose(Pose2d robotPose) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      if (robotPose.getY() > 4) {
        return RED_LEFT_POSE2D;
      } else {
        return RED_RIGHT_POSE2D;
      }
    } else {
      if (robotPose.getY() < 4) {
        return BLUE_LEFT_POSE2D;
      } else {
        return BLUE_RIGHT_POSE2D;
      }
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
              Logger.recordOutput("test/targetTurretRotations", rotations);
            },
            turret)
        .withName("turret aim");
  }

  private static Command turretLogicalAimCommand(
      Turret turret, Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> hubPoseSupplier) {

    return Commands.run(
            () -> {
              Pose2d rp = robotPoseSupplier.get();
              Pose2d hp =
                  (rp.getY() < 5.2 || rp.getY() > 10.8) ? hubPoseSupplier.get() : getSidePose(rp);
              double angleRelative = getAngleRelativeToHub(rp, hp);
              double rotations = angleRelative / (2 * Math.PI);
              turret.setPositionPID(rotations);
              Logger.recordOutput("test/targetTurretRotations", rotations);
            },
            turret)
        .withName("turret logical aim");
  }

  public static Command AimEverything(Turret turret, Hood hood, Supplier<Pose2d> poseSupplier) {
    // create a supplier that computes target RPS from the live robot pose

    Supplier<Pose2d> hubPoseSupplier = ShooterCommands::getAllianceHubPose;

    // While shooting, continuously aim the turret and hold the hood at the desired angle.
    // The flywheel shoot command acts as the deadline: when it finishes, aiming/hood are
    // interrupted.
    return Commands.run(
            () -> {
              Pose2d rp = poseSupplier.get();
              Pose2d hp = hubPoseSupplier.get();
              double angleRelative = getAngleRelativeToHub(rp, hp);
              double rotations = angleRelative / (2 * Math.PI);
              turret.setPositionPID(rotations);
            },
            turret)
        // .until(turret.isFinished())

        // hood.setPositionPIDCommand(theta))
        .withName("AimEverythingNoHood");
  }

  public static Command AimEverythingToHub(
      Turret turret, Hood hood, Supplier<Pose2d> poseSupplier, double theta) {
    // create a supplier that computes target RPS from the live robot pose

    Supplier<Pose2d> hubPoseSupplier = ShooterCommands::getAllianceHubPose;

    // While shooting, continuously aim the turret and hold the hood at the desired angle.
    // The flywheel shoot command acts as the deadline: when it finishes, aiming/hood are
    // interrupted.
    return Commands.parallel(
            Commands.run(
                () -> {
                  Pose2d rp = poseSupplier.get();
                  Pose2d hp = hubPoseSupplier.get();
                  double angleRelative = getAngleRelativeToHub(rp, hp);
                  double rotations = angleRelative / (2 * Math.PI);
                  turret.setPositionPID(rotations);
                },
                turret)
            // .until(turret.isFinished())
            ,
            Commands.run(() -> hood.setPositionPID(theta)))
        // hood.setPositionPIDCommand(theta))
        .withName("AimEverythingToHub");
  }

  // Shoots while adjusting flywheel speed based on distance
  public static Command ShootFromDistance(
      Flywheel flywheel,
      Hood hood,
      Tunnel tunnel,
      Hopper hopper,
      Intake intake,
      Supplier<Pose2d> poseSupplier,
      double theta) {
    // create a supplier that computes target RPS from the live robot pose
    Supplier<Double> targetRpsSupplier =
        () -> {
          Pose2d rp = poseSupplier.get();
          Pose2d hp = getAllianceHubPose();
          double liveXs = getDistanceToHub(rp, hp);
          Logger.recordOutput("Shooter/Distance", liveXs);
          Logger.recordOutput("Shooter/Pose", poseSupplier.get());
          Logger.recordOutput("Shooter/Pose", poseSupplier.get());

          if (liveXs > 15) {
            hood.setPositionPID(theta - 8);
            return calculateShooterRPS(liveXs, theta - 8);
          } else {
            hood.setPositionPID(theta);
            return calculateShooterRPS(liveXs, theta);
          }
        };

    return Commands.deadline(
            CommandFactory.shootCommand(flywheel, tunnel, hopper, intake, targetRpsSupplier))
        .withName("ShootFromDistance");
  }

  public static Command AimToSide(Turret turret, Supplier<Pose2d> poseSupplier) {
    Supplier<Pose2d> hubPoseSupplier = () -> getSidePose(poseSupplier.get());
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

  public static Command AimToHubOrSide(Turret turret, Supplier<Pose2d> poseSupplier) {
    Supplier<Pose2d> hubPoseSupplier = ShooterCommands::getAllianceHubPose;
    return turretLogicalAimCommand(turret, poseSupplier, hubPoseSupplier)
        .withName("turretAimToHub");
  }

  public static Pose3d getTurretPose(Supplier<Pose2d> poseSupplier) {
    Pose2d drivePose = poseSupplier.get();
    return new Pose3d(drivePose)
        .plus(
            new Transform3d(
                Inches.of(-8.708), Inches.of(8.299016), Inches.of(18.091), new Rotation3d()));
  }

  public static Pose3d getFlywheelPose(
      Supplier<Pose2d> poseSupplier, Supplier<Double> turretPosSupplier) {
    double turretAngleRads = turretPosSupplier.get() * 2 * Math.PI;
    return getTurretPose(poseSupplier)
        .plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, turretAngleRads)))
        .plus(
            new Transform3d(Inches.of(3.623), Inches.of(0), Inches.of(4.495771), new Rotation3d()));
  }
}
