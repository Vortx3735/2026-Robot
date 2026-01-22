package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Turret;
import java.util.function.Supplier;

public class TurretCommands {
  public static Command AimToHub(Turret turret, Supplier<Pose2d> poseSupplier) {
    return Commands.run(
        () -> {
          Pose2d robotPose = poseSupplier.get();
          Pose2d hubPose =
              DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red
                  ? new Pose2d(
                      Constants.FieldConstants.RED_HUB_POSE3D.getX(),
                      Constants.FieldConstants.RED_HUB_POSE3D.getY(),
                      new Rotation2d())
                  : new Pose2d(
                      Constants.FieldConstants.BLUE_HUB_POSE3D.getX(),
                      Constants.FieldConstants.BLUE_HUB_POSE3D.getY(),
                      new Rotation2d());
          double angleToHub =
              Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX());
          double robotYaw = robotPose.getRotation().getRadians();
          double angleRelative = angleToHub - robotYaw;
          // normalize to [-pi, pi]
          angleRelative = Math.atan2(Math.sin(angleRelative), Math.cos(angleRelative));
          double rotations = angleRelative / (2 * Math.PI);
          turret.setPositionPID(rotations);
        },
        turret);
  }

  public static Command AimToSide(Turret turret, Supplier<Pose2d> poseSupplier) {
    return Commands.run(
        () -> {
          Pose2d robotPose = poseSupplier.get();
          Pose2d hubPose =
              DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red
                  ? robotPose.getY() > 4.05
                      ? new Pose2d(
                          Constants.FieldConstants.RED_LEFT.getX(),
                          Constants.FieldConstants.RED_LEFT.getY(),
                          new Rotation2d())
                      : new Pose2d(
                          Constants.FieldConstants.RED_RIGHT.getX(),
                          Constants.FieldConstants.RED_RIGHT.getY(),
                          new Rotation2d())
                  : robotPose.getY() > 4.05
                      ? new Pose2d(
                          Constants.FieldConstants.BLUE_RIGHT.getX(),
                          Constants.FieldConstants.BLUE_RIGHT.getY(),
                          new Rotation2d())
                      : new Pose2d(
                          Constants.FieldConstants.BLUE_LEFT.getX(),
                          Constants.FieldConstants.BLUE_LEFT.getY(),
                          new Rotation2d());
          double angleToHub =
              Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX());
          double robotYaw = robotPose.getRotation().getRadians();
          double angleRelative = angleToHub - robotYaw;
          // normalize to [-pi, pi]
          angleRelative = Math.atan2(Math.sin(angleRelative), Math.cos(angleRelative));
          double rotations = angleRelative / (2 * Math.PI);
          turret.setPositionPID(rotations);
        },
        turret);
  }
}
