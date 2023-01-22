// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

public class TrajectoryUtils {
  public static PathPlannerTrajectory readTrajectory(String fileName, double MaxVelocity, double MaxAcceleration, boolean reverse) {

    if (fileName.startsWith("Red")) {
      try {
        return PathPlanner.loadPath(fileName, MaxVelocity, MaxAcceleration, reverse);
      } catch (Exception e) {
        // TODO: handle exception
        DriverStation.reportWarning("TrajectoryUtils::readTrajectory failed for " + fileName, null);
        fileName = fileName.replace("Red", "Blue");
        PathPlannerTrajectory trajectory = readTrajectory(fileName, MaxVelocity, MaxAcceleration, reverse);
        trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(
          trajectory, DriverStation.Alliance.Red);
        return trajectory;
      }
    } else {
      try {
        return PathPlanner.loadPath(fileName, MaxVelocity, MaxAcceleration, reverse);
      } catch (Exception e) {
        DriverStation.reportError("TrajectoryUtils::readTrajectory failed for " + fileName, null);
        return new PathPlannerTrajectory();
        // TODO: handle exception
      }
    }
  }
}
