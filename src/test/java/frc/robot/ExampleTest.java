package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

import org.junit.jupiter.api.*;

// Use @Disabled to disable a test class if it is still in progress. The "WIP" is a comment you can
// add for more details
@Disabled("WIP")
public class ExampleTest {
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

  // Mark all test functions with @Test
  @Test
  public void TestTest() {
    // Use assertEquals to check equality
    m_swerveDrive.setOdometry(new Pose2d());
    assertEquals(m_swerveDrive.getPoseMeters(), new Pose2d());

    // For booleans, you can use assertTrue/assertFalse
    assertTrue(m_swerveDrive.getModuleInitStatus());
  }
}
