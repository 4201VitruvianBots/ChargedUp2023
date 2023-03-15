// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static frc.robot.utils.VitruvianTrajectory.VitruvianState.transformStateForAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
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
      var file = new File(Filesystem.getDeployDirectory(), "pathplanner/" + fileName + ".path");
      if (!file.exists()) {
        DriverStation.reportWarning(
            "TrajectoryUtils::readTrajectory failed for " + fileName, false);
        fileName = fileName.replace("Red", "Blue");
      }

      var pathGroup = PathPlanner.loadPathGroup(fileName, pathConstraint, segmentConstraints);

      return flipTrajectory(pathGroup);
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

  public static List<PathPlannerTrajectory> flipTrajectory(List<PathPlannerTrajectory> trajectory) {
    return trajectory.stream()
        .map(TrajectoryUtils::transformTrajectoryForAlliance)
        .collect(Collectors.toList());
  }

  public static PathPlannerTrajectory transformTrajectoryForAlliance(
      PathPlannerTrajectory trajectory) {
    List<Trajectory.State> transformedStates = new ArrayList<>();

    for (Trajectory.State s : trajectory.getStates()) {
      PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) s;

      transformedStates.add(transformStateForAlliance(state));
    }

    return new PathPlannerTrajectory(
        transformedStates,
        trajectory.getMarkers(),
        trajectory.getStartStopEvent(),
        trajectory.getEndStopEvent(),
        trajectory.fromGUI);
  }

  //  public static PathPlannerTrajectory.PathPlannerState transformStateForAlliance(
  //          PathPlannerTrajectory.PathPlannerState state) {
  //    // Create a new state so that we don't overwrite the original
  //    PathPlannerTrajectory.PathPlannerState transformedState = new
  // PathPlannerTrajectory.PathPlannerState();
  //    Translation2d transformedTranslation =
  //            new Translation2d(
  //                    SimConstants.fieldLength - state.poseMeters.getX(),
  // state.poseMeters.getY());
  //    Rotation2d transformedHeading = new Rotation2d(-state.poseMeters.getRotation().getCos(),
  // state.poseMeters.getRotation().getSin());
  //    Rotation2d transformedHolonomicRotation = new Rotation2d(-state.holonomicRotation.getCos(),
  // state.holonomicRotation.getSin());
  //
  //    double stateCurveRadius = 0;
  //    double stateDeltaPos = 0;
  //
  //    try {
  //      // This is probably bad
  //      stateCurveRadius = (double) FieldUtils.readField(state, "curveRadius", true);
  //      stateDeltaPos = (double) FieldUtils.readField(state, "deltaPos", true);
  //    } catch (Exception e) {
  //
  //    }
  //    transformedState.timeSeconds = state.timeSeconds;
  //    transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
  //    transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
  //    transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
  //    transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
  //    transformedState.holonomicRotation = transformedHolonomicRotation;
  //    transformedState.holonomicAngularVelocityRadPerSec =
  // -state.holonomicAngularVelocityRadPerSec;
  ////      transformedState.curveRadius = -state.curveRadius;
  ////      transformedState.curveRadius = -state.curveRadius;
  //    transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
  ////      transformedState.deltaPos = state.deltaPos;
  //    transformedState.deltaPos = stateDeltaPos;
  //
  //    return transformedState;
  //  }
}
