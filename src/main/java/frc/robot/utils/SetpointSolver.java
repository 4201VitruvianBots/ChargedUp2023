package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.STATEHANDLER.WRIST_SETPOINT_OFFSET;

public class SetpointSolver {
  private static SetpointSolver m_instance;
  private Pose2d m_currentRobotPose;
  private Pose2d m_targetPose;
  private double m_targetTangentalOffset;
  private double m_wristOffset;

  private Rotation2d chassisSetpointDegrees;
  private double elevatorHorizontalSetpointMeters;
  private double elevatorSetpointMeters;

  private SetpointSolver() {}

  public static SetpointSolver getInstance() {
    if (m_instance == null) {
      m_instance = new SetpointSolver();
    }
    return m_instance;
  }

  public void solveSetpoints(
      Pose2d currentRobotPose, Pose2d targetPose, Constants.SCORING_STATE scoringState) {
    solveSetpoints(currentRobotPose, targetPose, scoringState, 0);
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
                scoringState == Constants.SCORING_STATE.SMART_LOW_REVERSE
                    ? Rotation2d.fromDegrees(180)
                    : Rotation2d.fromDegrees(0));

    switch (scoringState) {
      case SMART_LOW_REVERSE:
        m_wristOffset = 0;
        break;
      case SMART_LOW:
        m_wristOffset = WRIST_SETPOINT_OFFSET.LOW.get();
        break;
      case SMART_MEDIUM:
        m_wristOffset = WRIST_SETPOINT_OFFSET.MID.get();
        break;
      default:
      case SMART_HIGH:
        m_wristOffset = WRIST_SETPOINT_OFFSET.HIGH.get();
        break;
    }

    elevatorHorizontalSetpointMeters = correctedSolution.getTranslation().getNorm() - m_wristOffset;
    elevatorSetpointMeters =
        Math.cos(Constants.ELEVATOR.elevatorMountAngle.getRadians())
            * elevatorHorizontalSetpointMeters;
  }

  public Rotation2d getChassisSetpointRotation2d() {
    return chassisSetpointDegrees;
  }

  public double getElevatorSetpointMeters() {
    return elevatorSetpointMeters;
  }
}
