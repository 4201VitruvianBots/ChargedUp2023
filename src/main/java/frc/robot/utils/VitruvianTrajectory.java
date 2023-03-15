package frc.robot.utils;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.simulation.SimConstants;
import java.util.List;
import org.apache.commons.lang3.reflect.FieldUtils;

public class VitruvianTrajectory extends PathPlannerTrajectory {
  private final double m_totalTimeSeconds;
  private final List<State> m_states;
  private final List<EventMarker> markers;
  private final StopEvent startStopEvent;
  private final StopEvent endStopEvent;
  public final boolean fromGUI;

  public VitruvianTrajectory(PathPlannerTrajectory trajectory) {
    this(
        trajectory.getStates(),
        trajectory.getMarkers(),
        trajectory.getStartStopEvent(),
        trajectory.getEndStopEvent(),
        trajectory.fromGUI);
  }

  public VitruvianTrajectory(
      List<State> states,
      List<EventMarker> markers,
      StopEvent startStopEvent,
      StopEvent endStopEvent,
      boolean fromGUI) {
    super();
    this.m_states = states;
    this.m_totalTimeSeconds = m_states.get(m_states.size() - 1).timeSeconds;
    this.markers = markers;
    this.startStopEvent = startStopEvent;
    this.endStopEvent = endStopEvent;
    this.fromGUI = fromGUI;
  }

  /**
   * Get the "stop event" for the beginning of the path
   *
   * @return The start stop event
   */
  public StopEvent getStartStopEvent() {
    return this.startStopEvent;
  }

  /**
   * Get the "stop event" for the end of the path
   *
   * @return The end stop event
   */
  public StopEvent getEndStopEvent() {
    return this.endStopEvent;
  }

  /**
   * Get all of the markers in the path
   *
   * @return List of the markers
   */
  public List<EventMarker> getMarkers() {
    return this.markers;
  }
  //
  //    public static VitruvianTrajectory transformTrajectoryForAlliance(
  //            PathPlannerTrajectory trajectory) {
  //        List<State> transformedStates = new ArrayList<>();
  //
  //        for (Trajectory.State s : trajectory.getStates()) {
  //            PathPlannerTrajectory.PathPlannerState state =
  // (PathPlannerTrajectory.PathPlannerState) s;
  //
  //            transformedStates.add(transformStateForAlliance(state));
  //        }
  //
  //        return new VitruvianTrajectory(
  //                transformedStates,
  //                trajectory.getMarkers(),
  //                trajectory.getStartStopEvent(),
  //                trajectory.getEndStopEvent(),
  //                trajectory.fromGUI);
  //    }

  public static class VitruvianState extends PathPlannerState {
    public double curveRadius = 0;
    public double deltaPos = 0;

    public static PathPlannerState transformStateForAlliance(PathPlannerState state) {
      // Create a new state so that we don't overwrite the original
      VitruvianState transformedState = new VitruvianState();

      Translation2d transformedTranslation =
          new Translation2d(
              SimConstants.fieldLength - state.poseMeters.getX(), state.poseMeters.getY());
      Rotation2d transformedHeading =
          new Rotation2d(
              -state.poseMeters.getRotation().getCos(), state.poseMeters.getRotation().getSin());
      Rotation2d transformedHolonomicRotation =
          new Rotation2d(-state.holonomicRotation.getCos(), state.holonomicRotation.getSin());

      double stateCurveRadius = 0;
      double stateDeltaPos = 0;

      try {
        // This is probably bad
        stateCurveRadius = (double) FieldUtils.readField(state, "curveRadius", true);
        stateDeltaPos = (double) FieldUtils.readField(state, "deltaPos", true);
      } catch (Exception e) {

      }

      transformedState.timeSeconds = state.timeSeconds;
      transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
      transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
      transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
      transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
      transformedState.holonomicRotation = transformedHolonomicRotation;
      transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
      transformedState.curveRadius = -stateCurveRadius;
      transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
      transformedState.deltaPos = stateDeltaPos;

      return transformedState;
    }
  }
}
