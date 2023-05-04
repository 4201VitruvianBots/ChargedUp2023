package frc.robot.commands.auto;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

import org.junit.jupiter.api.*;

public class AutoTest {
  protected RobotContainer m_robotContainer;
  protected SwerveDrive m_swerveDrive;
  protected Elevator m_elevator;
  protected Wrist m_wrist;
  protected Intake m_intake;
  protected StateHandler m_stateHandler;
  protected Vision m_vision;
  protected FieldSim m_fieldSim;

  @BeforeEach
  // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_robotContainer = new RobotContainer();
    m_swerveDrive = m_robotContainer.getSwerveDrive();
    m_elevator = m_robotContainer.getElevator();
    m_wrist = m_robotContainer.getWrist();
    m_intake = m_robotContainer.getIntake();
    m_stateHandler = m_robotContainer.getStateHandler();
    m_vision = m_robotContainer.getVision();
    m_fieldSim = m_robotContainer.getFieldSim();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  // this method will run after each test
  void shutdown() throws Exception {
    m_robotContainer.close();
  }

  @Disabled("Cannot Flip PathPlanner")
  @Test
  public void testAutoPathFlipping() {
    System.out.println("Starting Test...");
    double blueTrajectoryMinX = 0;
    double blueTrajectoryMaxX = SimConstants.fieldLength / 2;
    double blueTrajectoryMinY = 0;
    double blueTrajectoryMaxY = SimConstants.fieldWidth - Units.inchesToMeters(90);
    var blueAuto =
        new SubstationTwo(
            "BlueSubstationTwo",
            m_swerveDrive,
            m_fieldSim,
            m_wrist,
            m_intake,
            m_vision,
            m_elevator,
            m_stateHandler);
    var blueTrajectories = blueAuto.getTrajectories();
    for (var segment : blueTrajectories) {
      for (var state : segment.getStates()) {
        assertTrue(
            blueTrajectoryMinX < state.poseMeters.getX()
                && state.poseMeters.getX() < blueTrajectoryMaxX);
        assertTrue(
            blueTrajectoryMinY < state.poseMeters.getY()
                && state.poseMeters.getY() < blueTrajectoryMaxY);
      }
    }

    double redTrajectoryMinX = SimConstants.fieldLength / 2;
    double redTrajectoryMaxX = SimConstants.fieldLength;
    double redTrajectoryMinY = Units.inchesToMeters(90);
    double redTrajectoryMaxY = SimConstants.fieldWidth;

    var redAuto =
        new SubstationTwo(
            "RedSubstationTwo",
            m_swerveDrive,
            m_fieldSim,
            m_wrist,
            m_intake,
            m_vision,
            m_elevator,
            m_stateHandler);
    var redTrajectories = redAuto.getTrajectories();
    for (var segment : redTrajectories) {
      for (var state : segment.getStates()) {
        assertTrue(
            redTrajectoryMinX < state.poseMeters.getX()
                && state.poseMeters.getX() < redTrajectoryMaxX);
        assertTrue(
            redTrajectoryMinY < state.poseMeters.getY()
                && state.poseMeters.getY() < redTrajectoryMaxY);
      }
    }
  }
}
