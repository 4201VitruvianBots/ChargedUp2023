package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.RobotContainer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveDriveTest {
  protected RobotContainer m_robotContainer;
  protected SwerveDrive m_swerveDrive;

  @BeforeEach
  // this method will run before each test. We Initialize the RobotContainer and get all subsystems
  // from it for our tests
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_robotContainer = new RobotContainer();
    m_swerveDrive = m_robotContainer.getSwerveDrive();
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
}
