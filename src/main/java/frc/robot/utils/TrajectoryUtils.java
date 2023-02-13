// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class TrajectoryUtils {
  public static List<PathPlannerTrajectory> readTrajectory(
      String fileName, PathConstraints segmentConstraints) {
    return readTrajectory(fileName, segmentConstraints, segmentConstraints);
  }

  public static List<PathPlannerTrajectory> readTrajectory(
      String fileName, PathConstraints pathConstraint, PathConstraints... segmentConstraints) {

    if (pathConstraint.maxVelocity == 0 || pathConstraint.maxAcceleration == 0) {
      DriverStation.reportError(fileName + " has an invalid velocity/acceleration", true);
    }
    for (var c : segmentConstraints) {
      if (c.maxVelocity == 0 || c.maxAcceleration == 0) {
        DriverStation.reportError(fileName + " has an invalid velocity/acceleration", true);
      }
    }

    if (fileName.startsWith("Red")) {
      try {
        return PathPlanner.loadPathGroup(fileName, pathConstraint, segmentConstraints);
      } catch (Exception e) {
        // TODO: handle exception
        DriverStation.reportWarning(
            "TrajectoryUtils::readTrajectory failed for " + fileName, e.getStackTrace());
        fileName = fileName.replace("Red", "Blue");
        List<PathPlannerTrajectory> trajectory =
            readTrajectory(fileName, pathConstraint, segmentConstraints);
        return trajectory.stream()
            .map(
                t ->
                    PathPlannerTrajectory.transformTrajectoryForAlliance(
                        t, DriverStation.Alliance.Red))
            .collect(Collectors.toList());
      }
    } else {
      try {
        return PathPlanner.loadPathGroup(fileName, pathConstraint, segmentConstraints);
      } catch (Exception e) {
        DriverStation.reportError("TrajectoryUtils::readTrajectory failed for " + fileName, null);
        return new ArrayList<PathPlannerTrajectory>();
        // TODO: handle exception
      }
    }
  }
}
