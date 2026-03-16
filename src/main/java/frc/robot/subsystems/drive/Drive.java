// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {

  private final PIDController m_pathXController = new PIDController(3.7, 0, 0);
  private final PIDController m_pathYController = new PIDController(3.7, 0, 0);
  private final PIDController m_pathThetaController = new PIDController(3.5, 0, 0);

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  // Maple-Sim config
  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
          .withCustomModuleTranslations(getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getFalcon500Foc(1),
                  TunerConstants.FrontLeft.DriveMotorGearRatio,
                  TunerConstants.FrontLeft.SteerMotorGearRatio,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  edu.wpi.first.units.Units.Meters.of(TunerConstants.FrontLeft.WheelRadius),
                  KilogramSquareMeters.of(0.03),
                  WHEEL_COF));

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  private final Consumer<Pose2d> resetSimulationPoseCallBack;
  // Guard to ensure AutoBuilder.configure is only invoked once per JVM. Some test
  // environments may construct Drive multiple times, which previously caused
  // PathPlanner's AutoBuilder.configure to log an error.
  private static boolean autoBuilderConfigured = false;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Configure AutoBuilder for PathPlanner (only once)
    if (!autoBuilderConfigured) {
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getChassisSpeeds,
          this::runVelocity,
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
          PP_CONFIG,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this);
      autoBuilderConfigured = true;
    }
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      Logger.recordOutput("pose", getPose());
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public Command stopCommand() {
    return runOnce(() -> this.stop());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  // ---------------------------------------------------------------------------
  // POSE GETTERS
  // ---------------------------------------------------------------------------

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  // ---------------------------------------------------------------------------
  // SHOOTER MECHANISM POSES
  //
  // Chain: Robot (Pose2d) → Turret Pivot (Pose3d) → Flywheel Exit (Pose3d)
  //
  // All offsets are in the WPILib robot coordinate system:
  //   +X = forward, +Y = left, +Z = up
  //   Angles follow right-hand rule (CCW positive looking down Z)
  //
  // Turret pivot offset is FIXED relative to the robot frame — it never changes.
  // Flywheel offset is relative to the turret pivot and ROTATES with the turret,
  // so when the turret yaws, the flywheel tip orbits around the pivot point.
  // ---------------------------------------------------------------------------

  /**
   * Returns the field-relative 3D pose of the turret pivot point.
   *
   * <p>This is a fixed mechanical offset from the robot center — the turret pivot
   * location bolted to the robot. It does NOT change with turret rotation.
   *
   * <p>Offset values (arbitrary, tune to match your robot CAD):
   *   x = +2 in forward of robot center
   *   y =  0 in (centered left-right)
   *   z = +18 in above the floor
   */
  @AutoLogOutput(key = "Shooter/TurretPivotPose")
  public Pose3d getTurretPivotPose() {
    // Step 1: Lift the 2D robot pose into 3D (z=0, no pitch/roll)
    Pose3d robotPose3d = new Pose3d(getPose());

    // Step 2: Apply the fixed robot-to-turret-pivot transform
    // Rotation3d() = identity — the pivot platform doesn't tilt relative to the robot
    Transform3d robotToTurretPivot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(2.0),  // +X: 2 inches forward of robot center
                Units.inchesToMeters(0.0),  // +Y: centered
                Units.inchesToMeters(18.0)  // +Z: 18 inches above floor
            ),
            new Rotation3d() // no rotation — pivot base is level with robot
        );

    return robotPose3d.plus(robotToTurretPivot);
  }

  /**
   * Returns the field-relative 3D pose of the flywheel exit point.
   *
   * <p>Chains two transforms: robot → turret pivot → flywheel tip.
   * The flywheel tip orbits the pivot as the turret rotates around Z (yaw).
   *
   * <p>Flywheel offset from turret pivot (arbitrary, tune to match CAD):
   *   x = +8 in in the turret's forward direction (before rotation applied)
   *   y =  0 in
   *   z = +4 in above the pivot
   *
   * @param turretRotations current turret position in rotations
   *     (from Turret.getTurretCurrentPosition())
   */
  public Pose3d getFlywheelPose(double turretRotations) {
    // Step 1: Get the turret pivot pose in field space
    Pose3d turretPivotPose = getTurretPivotPose();

    // Step 2: Convert turret rotations → radians for Rotation3d
    // Positive rotations = CCW when viewed from above (standard WPILib convention)
    double turretAngleRadians = turretRotations * 2.0 * Math.PI;

    // Step 3: Build turret-pivot → flywheel-tip transform
    // The Rotation3d here is what makes the flywheel orbit the pivot:
    //   when turretAngleRadians = 0   → flywheel is directly in front of pivot (+X)
    //   when turretAngleRadians = π/2 → flywheel is to the LEFT of pivot (+Y)
    //   when turretAngleRadians = π   → flywheel is behind pivot (-X)
    Transform3d turretToFlywheel =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(8.0), // +X: 8 inches in turret's local forward
                Units.inchesToMeters(0.0), // +Y: centered
                Units.inchesToMeters(4.0)  // +Z: 4 inches above pivot
            ),
            new Rotation3d(0.0, 0.0, turretAngleRadians) // yaw rotation of the turret
        );

    // Step 4: Apply the turret-relative transform to get field-space flywheel pose
    return turretPivotPose.plus(turretToFlywheel);
  }

  /**
   * Convenience overload — logs the flywheel pose with @AutoLogOutput by caching
   * the last turret angle set via {@link #updateFlywheelPose(double)}.
   *
   * <p>Call updateFlywheelPose(turret.getTurretCurrentPosition()) in RobotContainer
   * or Telemetry each loop, then this auto-logged getter will publish it.
   */
  private double cachedTurretRotations = 0.0;

  /**
   * Call this every loop (from RobotContainer or Telemetry) to feed the current
   * turret angle into Drive so the @AutoLogOutput getter can publish it.
   *
   * <p>Example in RobotContainer periodic:
   * <pre>
   *   drive.updateFlywheelPose(turret.getTurretCurrentPosition());
   * </pre>
   */
  public void updateFlywheelPose(double turretRotations) {
    cachedTurretRotations = turretRotations;
  }

  /**
   * Auto-logged flywheel pose — published every loop to AdvantageScope.
   * Requires {@link #updateFlywheelPose(double)} to be called each loop.
   * View this in AdvantageScope under key "Shooter/FlywheelPose" as a Pose3d.
   */
  @AutoLogOutput(key = "Shooter/FlywheelPose")
  public Pose3d getFlywheelPoseLogged() {
    return getFlywheelPose(cachedTurretRotations);
  }

  // ---------------------------------------------------------------------------
  // LEGACY HOOD POSE (kept for compatibility)
  // ---------------------------------------------------------------------------

  /** Returns the hood pose (old method — kept for backward compat). */
  @AutoLogOutput(key = "Shooter/HoodPose")
  public Pose3d getHoodPose() {
    Pose2d drivePose = poseEstimator.getEstimatedPosition();
    // robot to hood transform  x:-10.135-0.51 y:13.866-2.885 z:18.126 (inches)
    return new Pose3d(drivePose)
        .plus(
            new Transform3d(
                Inches.of(-(10.135 - 0.51)),
                Inches.of(13.866 - 4.385),
                Inches.of(14.126),
                new Rotation3d()));
  }

  // ---------------------------------------------------------------------------
  // ODOMETRY RESET / FOLLOW PATH / VISION
  // ---------------------------------------------------------------------------

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void zeroDriveTrain() {
    Pose2d zeroedPose = new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0));
    resetOdometry(zeroedPose);
  }

  public void followPath(SwerveSample sample) {
    var pose = getPose();

    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        m_pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    runVelocity(targetSpeeds);
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  // ---------------------------------------------------------------------------
  // AUTO FACTORIES
  // ---------------------------------------------------------------------------

  public AutoFactory createAutoFactory() {
    return createAutoFactory((sample, isStart) -> {});
  }

  public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
    return new AutoFactory(
        () -> getPose(), this::resetOdometry, this::followPath, true, this, trajLogger);
  }

  // ---------------------------------------------------------------------------
  // SPEED / GEOMETRY HELPERS
  // ---------------------------------------------------------------------------

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}