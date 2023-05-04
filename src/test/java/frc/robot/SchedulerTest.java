package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

// Use @Disabled to disable a test class if it is still in progress. The "WIP" is a comment you can
// add for more details
@Disabled
public class SchedulerTest extends CommandTestBase {
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
  public void TestCommandScheduling() {
    var testCommand = new PrintCommand("Hello World").andThen(new PrintCommand("Hello World 2"));
    testCommand.addRequirements(m_swerveDrive);
    CommandScheduler.getInstance().schedule(testCommand);
    //    CommandScheduler.getInstance().run();

    assertEquals(m_swerveDrive.getCurrentCommand(), testCommand);
  }
}
