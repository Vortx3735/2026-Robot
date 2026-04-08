// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.ProjectileSimulator;
import frc.robot.util.ProjectileSimulator.GeneratedLUT;
import frc.robot.util.ProjectileSimulator.LUTEntry;
import frc.robot.util.ProjectileSimulator.SimParameters;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);

    SimParameters params = new SimParameters(
       0.215,   // ball mass kg
       0.1501,  // ball diameter m
       0.47,    // drag coeff (smooth sphere)
       0.2,     // Magnus coeff
       1.225,   // air density kg/m^3
       0.43,    // exit height from floor, measure from CAD (top of the flywheel)
       0.1016,  // wheel diameter, measure with calipers 
       1.83,    // target height, from game manual
       0.8,     // slip factor (0=no grip, 1=perfect), tune on robot 
       65.0,    // launch angle degrees from horizontal
       0.001,   // sim timestep
       1500, 6000, 25, 5.0  // RPM range, search iters, max sim time
   );
   ProjectileSimulator sim = new ProjectileSimulator(params);
   GeneratedLUT lut = sim.generateLUT();
   for (LUTEntry entry : lut.entries()) {
       if (entry.reachable()) {
           System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
               entry.distanceM(), entry.rpm(), entry.tof());
       }
   }
  }
}
