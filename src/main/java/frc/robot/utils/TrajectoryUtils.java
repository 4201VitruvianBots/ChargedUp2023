// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

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
      var file = new File(Filesystem.getDeployDirectory(), "pathplanner/" + fileName + ".path");
      if (!file.exists()) {
        DriverStation.reportWarning(
            "TrajectoryUtils::readTrajectory failed for " + fileName, false);
        fileName = fileName.replace("Red", "Blue");

        var pathGroup = PathPlanner.loadPathGroup(fileName, pathConstraint, segmentConstraints);

        ArrayList<PathPlannerTrajectory> ppTrajectories = new ArrayList<>();
        for (var trajectory : pathGroup) {
          ppTrajectories.add(
              PathPlannerTrajectory.transformTrajectoryForAlliance(
                  trajectory, DriverStation.Alliance.Red));
        }
        return ppTrajectories;
      }
      return PathPlanner.loadPathGroup(fileName, pathConstraint, segmentConstraints);
    } else {
      try {
        var file = new File(Filesystem.getDeployDirectory(), "pathplanner/" + fileName + ".path");

        return PathPlanner.loadPathGroup(fileName, pathConstraint, segmentConstraints);
      } catch (Exception e) {
        DriverStation.reportError("TrajectoryUtils::readTrajectory failed for " + fileName, null);
        return new ArrayList<>();
      }
    }
  }
}

// TODO: Fix after L.A.
// public class TrajectoryUtils {
//  public static List<PathPlannerTrajectory> readTrajectory(
//      String fileName, PathConstraints segmentConstraints) {
//    return readTrajectory(fileName, segmentConstraints, segmentConstraints);
//  }
//
//  public static List<PathPlannerTrajectory> readTrajectory(
//      String fileName, PathConstraints pathConstraint, PathConstraints... segmentConstraints) {
//
//    if (pathConstraint.maxVelocity == 0 || pathConstraint.maxAcceleration == 0) {
//      DriverStation.reportError(fileName + " has an invalid velocity/acceleration", true);
//    }
//    for (var c : segmentConstraints) {
//      if (c.maxVelocity == 0 || c.maxAcceleration == 0) {
//        DriverStation.reportError(fileName + " has an invalid velocity/acceleration", true);
//      }
//    }
//
//    if (fileName.startsWith("Red")) {
//      var file = new File(Filesystem.getDeployDirectory(), "pathplanner/" + fileName + ".path");
//      if (!file.exists()) {
//        DriverStation.reportWarning(
//            "TrajectoryUtils::readTrajectory failed for " + fileName, false);
//        fileName = fileName.replace("Red", "Blue");
//      }
//
//      var pathGroup = PathPlanner.loadPathGroup(fileName, pathConstraint, segmentConstraints);
//
//      return flipTrajectory(pathGroup);
//    } else {
//      try {
//        var file = new File(Filesystem.getDeployDirectory(), "pathplanner/" + fileName + ".path");
//
//        return PathPlanner.loadPathGroup(fileName, pathConstraint, segmentConstraints);
//      } catch (Exception e) {
//        DriverStation.reportError("TrajectoryUtils::readTrajectory failed for " + fileName, null);
//        return new ArrayList<>();
//      }
//    }
//  }
//
//  public static List<PathPlannerTrajectory> flipTrajectory(List<PathPlannerTrajectory> trajectory)
// {
//    return trajectory.stream()
//        .map(TrajectoryUtils::transformTrajectoryForAlliance)
//        .collect(Collectors.toList());
//  }
//
//  public static PathPlannerTrajectory transformTrajectoryForAlliance(
//      PathPlannerTrajectory trajectory) {
//    List<Trajectory.State> transformedStates = new ArrayList<>();
//
//    for (Trajectory.State s : trajectory.getStates()) {
//      PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) s;
//
//      transformedStates.add(transformStateForAlliance(state));
//    }
//
//    return new PathPlannerTrajectory(
//        transformedStates,
//        trajectory.getMarkers(),
//        trajectory.getStartStopEvent(),
//        trajectory.getEndStopEvent(),
//        trajectory.fromGUI);
//  }
// }
