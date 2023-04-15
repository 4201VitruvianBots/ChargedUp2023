package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static utils.TestUtils.setPrivateField;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CommandTestBase;
import frc.robot.Constants.USB;
import frc.robot.RobotContainer;
import frc.robot.simulation.SimConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

public class SwerveDriveTest extends CommandTestBase {
  protected RobotContainer m_robotContainer;
  protected SwerveDrive m_swerveDrive;
  protected Controls m_controls;

  @BeforeEach
  // this method will run before each test. We Initialize the RobotContainer and get all subsystems
  // from it for our tests
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_robotContainer = new RobotContainer();
    m_swerveDrive = m_robotContainer.getSwerveDrive();
    m_controls = m_robotContainer.getControls();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  // this method will run after each test. We need to close each subsystem properly, as they all get
  // separately initialized for each individual test, which will trip some errors due to how WPILib
  // is set up (e.g. resource errors from using the same PWM/DIO port)
  void shutdown() throws Exception {
    m_robotContainer.close();
  }

  @Test
  public void TestPIDControllers() {
    var xPidController = m_swerveDrive.getXPidController();
    var yPidController = m_swerveDrive.getYPidController();
    var tPidController = m_swerveDrive.getThetaPidController();
    double output;

    output = xPidController.calculate(0, 1);
    assertTrue(output > 0);
    output = xPidController.calculate(0, -1);
    assertTrue(output < 0);

    output = yPidController.calculate(0, 1);
    assertTrue(output > 0);
    output = yPidController.calculate(0, -1);
    assertTrue(output < 0);

    output = tPidController.calculate(0, 1);
    assertTrue(output > 0);
    output = tPidController.calculate(0, -1);
    assertTrue(output < 0);
    output = tPidController.calculate(-1, 1);
    assertTrue(output > 0);
  }

  @Disabled
  @Test
  public void TestAllianceFlipTeleop() {
    setPrivateField(m_controls, "allianceColor", DriverStation.Alliance.Red);
    m_swerveDrive.setOdometry(
        new Pose2d(SimConstants.fieldLength, 0, Rotation2d.fromDegrees(-180)));

    for (int i = 0; i < 10; i++) {
      DriverStationSim.setJoystickAxis(USB.leftJoystick, 1, 0.5);
      CommandScheduler.getInstance().run();
    }
    System.out.println(m_swerveDrive.getOdometry().getEstimatedPosition().getX());
    assertTrue(
        m_swerveDrive.getOdometry().getEstimatedPosition().getX() < SimConstants.fieldLength);
  }
}
