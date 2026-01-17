package frc.robot.util;

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
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {}

  public static class FlywheelConstants {
    public static final int flywheelId = 1;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 13;
  }

  public static class ClimberConstants {}

  public static class TurretConstants {
    public static final int turretId = 1;
  }
  
  public static class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = 17;
  }
}
