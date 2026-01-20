package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Turret;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class TurretDefaultCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Turret turret;

  private final Supplier<Pose2d> poseSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurretDefaultCommand(Turret subsystem, Supplier<Pose2d> poseSupplier) {
    turret = subsystem;
    this.poseSupplier = poseSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = poseSupplier.get();
    Pose2d hubPose =
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
