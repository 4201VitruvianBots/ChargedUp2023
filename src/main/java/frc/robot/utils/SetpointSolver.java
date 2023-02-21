package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.Constants;

public class SetpointSolver {
  private SetpointSolver m_instance;
  private Pose2d m_currentRobotPose;
  private Pose2d m_targetPose;
  private double m_targetTangentalOffset;
  private double m_wristOffset;

  private Rotation2d chassisSetpointDegrees;
  private double elevatorHorizontalSetpointMeters;
  private double elevatorSetpointMeters;

  private SetpointSolver() {}

  public SetpointSolver getInstance() {
    if (m_instance == null) {
      m_instance = new SetpointSolver();
    }
    return m_instance;
  }

  public void solveSetpoints(
      Pose2d currentRobotPose, Pose2d targetPose, Constants.SCORING_STATES scoringState) {
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
      Constants.SCORING_STATES scoringState,
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
                scoringState == Constants.SCORING_STATES.LOW_INTAKE
                    ? Rotation2d.fromDegrees(180)
                    : Rotation2d.fromDegrees(0));

    switch (scoringState) {
      case LOW_INTAKE:
        m_wristOffset = 0;
        break;
      case LOW:
        m_wristOffset = Constants.SetpointSolver.WRIST_HORIZONTAL_LOW_OFFSET;
        break;
      case MID:
        m_wristOffset = Constants.SetpointSolver.WRIST_HORIZONTAL_MID_OFFSET;
        break;
      default:
      case HIGH:
        m_wristOffset = Constants.SetpointSolver.WRIST_HORIZONTAL_HIGH_OFFSET;
        break;
    }

    elevatorHorizontalSetpointMeters = correctedSolution.getTranslation().getNorm() - m_wristOffset;
    elevatorSetpointMeters =
        Math.cos(Constants.getInstance().Elevator.elevatorMountAngle.getRadians())
            * elevatorHorizontalSetpointMeters;
  }

  public Rotation2d getChassisSetpointRotation2d() {
    return chassisSetpointDegrees;
  }

  public double getElevatorSetpointMeters() {
    return elevatorSetpointMeters;
  }
}
