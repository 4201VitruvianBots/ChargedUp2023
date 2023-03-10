package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class SetpointSolver {
  private static SetpointSolver m_instance;
  private Pose2d m_currentRobotPose;
  private Pose2d m_targetPose;
  private double m_targetTangentalOffset;
  private double m_wristOffset;

  private Rotation2d chassisSetpointDegrees;
  private double elevatorHorizontalSetpointMeters;
  private double elevatorSetpointMeters;
  private double wristSetpointDegrees;

  private SetpointSolver() {}

  public static SetpointSolver getInstance() {
    if (m_instance == null) {
      m_instance = new SetpointSolver();
    }
    return m_instance;
  }

  public void solveSetpoints(
      Pose2d currentRobotPose,
      Pose2d targetPose,
      double wristOffset,
      Constants.SCORING_STATE scoringState) {
    solveSetpoints(currentRobotPose, targetPose, wristOffset, scoringState, 0);
  }

  /**
   * Given the current robot pose and a valid target pose, find the closest pose and calculate the
   * desired setpoints for the chassis heading and elevator height. Assume fixed angle setpoints for
   * the wrist, which will determine the max elevator extension. Can also take into account the
   * approximate offset of the game object within the intake.
   */
  public void solveSetpoints(
      Pose2d currentRobotPose,
      Pose2d targetPose,
      double wristOffset,
      Constants.SCORING_STATE scoringState,
      double targetTangentalOffset) {
    m_currentRobotPose = currentRobotPose;
    m_targetPose = targetPose;
    m_targetTangentalOffset = targetTangentalOffset;

    var solution = m_targetPose.relativeTo(m_currentRobotPose);
    var solutionTransform =
        new Transform2d(new Translation2d(0, m_targetTangentalOffset), Rotation2d.fromDegrees(90));

    var correctedSolution = solution.transformBy(solutionTransform);

    chassisSetpointDegrees =
        correctedSolution
            .getRotation()
            .plus(
                scoringState == Constants.SCORING_STATE.LOW_REVERSE
                    ? Rotation2d.fromDegrees(180)
                    : Rotation2d.fromDegrees(0));

    elevatorHorizontalSetpointMeters = correctedSolution.getTranslation().getNorm() - wristOffset;
    elevatorSetpointMeters =
        Math.cos(Constants.ELEVATOR.mountAngleRadians.getRadians())
            * elevatorHorizontalSetpointMeters;
  }

  public Rotation2d getChassisSetpointRotation2d() {
    return chassisSetpointDegrees;
  }

  public double getElevatorSetpointMeters() {
    return elevatorSetpointMeters;
  }

  public double getWristSetpointDegrees() {
    return wristSetpointDegrees;
  }
}
