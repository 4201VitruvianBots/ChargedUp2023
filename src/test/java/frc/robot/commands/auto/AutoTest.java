package frc.robot.commands.auto;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.*;
import org.junit.jupiter.api.Test;

public class AutoTest {
  protected final RobotContainer m_robotContainer = new RobotContainer();
  protected final SwerveDrive m_swerveDrive = m_robotContainer.getSwerveDrive();
  protected final Elevator m_elevator = m_robotContainer.getElevator();
  protected final Wrist m_wrist = m_robotContainer.getWrist();
  protected final Intake m_intake = m_robotContainer.getIntake();
  protected final Vision m_vision = m_robotContainer.getVision();
  protected final FieldSim m_fieldSim = m_robotContainer.getFieldSim();
  protected final SwerveAutoBuilder m_autoBuilder = m_robotContainer.getAutoBuilder();

  @Test
  public void AutoTest() {
    testAutoPathFipping();
  }

  @Test
  public void testAutoPathFipping() {
    System.out.println("Starting Test...");
    double blueTrajectoryMinX = 0;
    double blueTrajectoryMaxX = SimConstants.fieldLength / 2;
    double blueTrajectoryMinY = 0;
    double blueTrajectoryMaxY = SimConstants.fieldWidth - Units.inchesToMeters(90);
    var blueAuto =
        new OnePiece(
            "BlueOnePiece", m_autoBuilder, m_swerveDrive, m_fieldSim, m_wrist, m_intake, m_vision);
    var blueTrajectories = blueAuto.getTrajectory();
    for (var segment : blueTrajectories) {
      for (var state : segment.getStates()) {
        assert (blueTrajectoryMinX < state.poseMeters.getX()
            && state.poseMeters.getX() < blueTrajectoryMaxX);
        assert (blueTrajectoryMinY < state.poseMeters.getY()
            && state.poseMeters.getY() < blueTrajectoryMaxY);
      }
    }

    double redTrajectoryMinX = SimConstants.fieldLength / 2;
    double redTrajectoryMaxX = SimConstants.fieldLength;
    double redTrajectoryMinY = Units.inchesToMeters(90);
    double redTrajectoryMaxY = SimConstants.fieldWidth;

    //        var test = TrajectoryUtils.readTrajectory("RedOnePiece", new PathConstraints(1, 1));

    var redAuto =
        new OnePiece(
            "RedOnePiece", m_autoBuilder, m_swerveDrive, m_fieldSim, m_wrist, m_intake, m_vision);
    var redTrajectories = redAuto.getTrajectory();
    for (var segment : redTrajectories) {
      for (var state : segment.getStates()) {
        assert (redTrajectoryMinX < state.poseMeters.getX()
            && state.poseMeters.getX() < redTrajectoryMaxX);
        assert (redTrajectoryMinY < state.poseMeters.getY()
            && state.poseMeters.getY() < redTrajectoryMaxY);
      }
    }
  }
}
