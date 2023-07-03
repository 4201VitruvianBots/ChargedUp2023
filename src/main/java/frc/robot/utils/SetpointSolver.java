package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.Constants.WRIST;

public class SetpointSolver {
  private static SetpointSolver m_instance;
  private Pose2d m_currentRobotPose;
  private Pose2d m_targetPose;
  private double m_targetTangentalOffset;

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
      Pose2d currentRobotPose, Pose2d targetPose, double wristOffset, SCORING_STATE scoringState) {
    solveSetpoints(currentRobotPose, targetPose, wristOffset, scoringState, 0);
  }

  public boolean canScore() {
    return ELEVATOR.THRESHOLD.ALPHA_MIN.get() <= elevatorSetpointMeters
        && elevatorSetpointMeters <= ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get()
        && Units.radiansToDegrees(WRIST.THRESHOLD.ABSOLUTE_MIN.get()) <= wristSetpointDegrees
        && wristSetpointDegrees <= Units.radiansToDegrees(WRIST.THRESHOLD.ABSOLUTE_MAX.get());
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
      SCORING_STATE scoringState,
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
                scoringState == SCORING_STATE.LOW_REVERSE
                    ? Rotation2d.fromDegrees(180)
                    : Rotation2d.fromDegrees(0));

    // TODO: Calculate wrist offset? Otherwise use fixed offset by measured setpoints
    elevatorHorizontalSetpointMeters = correctedSolution.getTranslation().getNorm() - wristOffset;
    elevatorSetpointMeters =
        Math.cos(ELEVATOR.mountAngleRadians.getRadians()) * elevatorHorizontalSetpointMeters;
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
