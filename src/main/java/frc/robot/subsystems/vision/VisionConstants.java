// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String frontCameraName = "front";
  public static String backCameraName = "back";
  public static String rightCameraName = "right";
  public static String leftCameraName = "left";

  // note: zero z component might be 0.5 meters above the ground

  // note: zero z component might be 0.5 meters above the ground
  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d frontCameraTransform =
      new Transform3d(
          Inches.of(12.671076),
          Inches.of(-0.499150),
          Inches.of(15.651053),
          new Rotation3d(0.0, -18.0 * Math.PI / 180, 0.0));
  public static Transform3d backCameraTransform =
      new Transform3d(
          Inches.of(-9.646),
          Inches.of(-3.844),
          Inches.of(18.190),
          new Rotation3d(0.0, -30.0 * Math.PI / 180, 0.0)
              .rotateBy(new Rotation3d(0.0, 0.0, Math.PI)));
  public static Transform3d rightCameraTransform =
      new Transform3d(
          Inches.of(-2.458),
          Inches.of(-14.702),
          Inches.of(21.460),
          new Rotation3d(0.0, -30.0 * Math.PI / 180, 0.0)
              .rotateBy(new Rotation3d(0.0, 0.0, Math.PI * 3.0 / 2.0)));
  public static Transform3d leftCameraTransform =
      new Transform3d(
          Inches.of(-2.594),
          Inches.of(13.962),
          Inches.of(19.468),
          new Rotation3d(0.0, -30.0 * Math.PI / 180, 0.0)
              .rotateBy(new Rotation3d(0.0, 0.0, Math.PI / 2.0)));

  // Basic filtering threshold
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
