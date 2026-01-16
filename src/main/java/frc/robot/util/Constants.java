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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {}

  public static class FlywheelConstants {
    public static final int flywheelId = 1;
  }

  public static class IntakeConstants {}

  public static class ClimberConstants {}

  public static class IndexerConstants {}

  public static class TurretConstants {
    public static final int turretId = 1;
  }
}
