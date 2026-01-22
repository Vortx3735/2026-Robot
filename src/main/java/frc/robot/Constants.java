package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveTrainConstants { // Id's are all placeholders
    public static final int FRONTLEFT_DRIVE_ID = 1;
    public static final int FRONTLEFT_TURN_ID = 2;
    public static final int FRONTLEFT_CANCODER = 3;

    public static final int FRONTRIGHT_DRIVE_ID = 4;
    public static final int FRONTRIGHT_TURN_ID = 5;
    public static final int FRONTRIGHT_CANCODER = 6;

    public static final int BACKLEFT_DRIVE_ID = 7;
    public static final int BACKLEFT_TURN_ID = 8;
    public static final int BACKLEFT_CANCODER = 9;

    public static final int BACKRIGHT_DRIVE_ID = 10;
    public static final int BACKRIGHT_TURN_ID = 11;
    public static final int BACKRIGHT_CANCODER = 12;

    public static final int PIGEON_ID = 23;
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class DriveConstants {}

  public static class FlywheelConstants {
    public static final int FLYWHEEL_MOTOR_ID = 1;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 13;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 69;
  }

  public static class TurretConstants {
    public static final int TURRET_MOTOR_ID = 1;
  }

  public static class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = 17;
  }

  public static class FieldConstants {
    public static final double DISTANCE_X_TO_HUB = 4.6736;
    public static final Pose3d BLUE_HUB_POSE3D =
        new Pose3d(DISTANCE_X_TO_HUB, 4.05, 1.8, new Rotation3d());
    public static final Pose3d RED_HUB_POSE3D =
        new Pose3d(16.5736 - DISTANCE_X_TO_HUB, 4.05, 1.8, new Rotation3d());
    public static final Pose2d BLUE_RIGHT = new Pose2d(3.09, 7.10, new Rotation2d());
    public static final Pose2d BLUE_LEFT = new Pose2d(3.09, 1.39, new Rotation2d());
    public static final Pose2d RED_RIGHT = new Pose2d(13.78, 1.39, new Rotation2d());
    public static final Pose2d RED_LEFT = new Pose2d(13.78, 7.10, new Rotation2d());
  }
}
