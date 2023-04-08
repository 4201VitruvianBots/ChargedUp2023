package frc.robot.commands.auto;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CommandTestBase;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.RobotContainer;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.TrajectoryUtils;
import org.junit.jupiter.api.*;

@Disabled
public class AutoTest extends CommandTestBase {
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

  @Test
  public void testAutoPathFlipping() {
    System.out.println("Starting Test...");
    double blueTrajectoryMinX = 0;
    double blueTrajectoryMaxX = SimConstants.fieldLength / 2;
    double blueTrajectoryMinY = 0;
    double blueTrajectoryMaxY = SimConstants.fieldWidth - Units.inchesToMeters(90);
    var blueAuto =
        new TwoPiece(
            "BlueTwoPiece",
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

    //        var test = TrajectoryUtils.readTrajectory("RedOnePiece", new PathConstraints(1, 1));

    var redAuto =
        new TwoPiece(
            "RedTwoPiece",
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

  @Test
  public void testAutoPathFollowing() {

    var trajectories =
        TrajectoryUtils.readTrajectory(
            "TestSimAuto Copy",
            new PathConstraints(
                SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.4,
                SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.4));
    var commands = TrajectoryUtils.generatePPSwerveControllerCommand(m_swerveDrive, trajectories);

    var timer = new Timer();
    m_swerveDrive.setOdometry(trajectories.get(0).getInitialHolonomicPose());

    for (int i = 0; i < trajectories.size(); i++) {
      CommandScheduler.getInstance().schedule(commands.get(i));
      double currentTime;
      timer.reset();
      timer.start();

      while (true) {
        CommandScheduler.getInstance().run();
        currentTime = timer.get();
        if (trajectories.get(i).getTotalTimeSeconds() < currentTime) {
          break;
        }

        var currentPose = m_swerveDrive.getPoseMeters();
        var trajectoryPose = trajectories.get(i).sample(currentTime);

        var xDelta = Math.abs(trajectoryPose.poseMeters.getX() - currentPose.getX());
        var yDelta = Math.abs(trajectoryPose.poseMeters.getY() - currentPose.getY());
        if (xDelta > 2 || yDelta > 2) {
          System.out.printf("Trajectory failed at T=%fs", currentTime);
          assertTrue(false);
        }
      }
    }
    assertTrue(true);
  }
}
