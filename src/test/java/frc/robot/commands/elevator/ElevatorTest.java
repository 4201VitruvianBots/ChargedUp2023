package frc.robot.commands.elevator;

import static org.junit.jupiter.api.Assertions.*;
import static utils.TestUtils.setPrivateField;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants.ELEVATOR;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import org.junit.jupiter.api.*;

// Use @Disabled to disable a test class if it is still in progress. The "WIP" is a comment you can
// add for more details
@Disabled("WIP")
public class ElevatorTest {
  protected RobotContainer m_robotContainer;
  protected Elevator m_elevator;

  @BeforeEach
  // this method will run before each test. We Initialize the RobotContainer and get all subsystems
  // from it for our tests
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_robotContainer = new RobotContainer();
    m_elevator = m_robotContainer.getElevator();

    setPrivateField(m_elevator, "m_unitTesting", true);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  // this method will run after each test. We need to close each subsystem properly, as they all get
  // separately initialized for each individual test, which will trip some errors due to how WPILib
  // is set up (e.g. resource errors from using the same PWM/DIO port)
  void shutdown() throws Exception {
    m_robotContainer.close();
    m_elevator.close();
  }

  // Test whether the setpoint we input gets set as the elevator's setpoint.
  @Test
  public void TestSetSetpoint() {
    var setpointCommand =
        new SetElevatorSetpoint(m_elevator, ELEVATOR.SETPOINT.SCORE_LOW_CONE.get());

    setpointCommand.initialize();
    // setPrivateField(m_elevator, "m_currentTimestamp", 3.0);

    // double distanceBetween =
    //     Math.abs(ELEVATOR.SETPOINT.SCORE_LOW_CONE.get() - m_elevator.getHeightMeters());

    assertEquals(ELEVATOR.SETPOINT.SCORE_LOW_CONE.get(), m_elevator.getDesiredPositionMeters());
  }

  // Test both that we can manually set the elevator's reported position and have the ability to
  // reset it via a command.
  @Test
  public void TestReset() {
    var resetCommand = new ResetElevatorHeight(m_elevator, 0);

    m_elevator.setSensorPosition(1);
    assertEquals(1, m_elevator.getHeightMeters());

    resetCommand.initialize();

    assertEquals(0, m_elevator.getHeightMeters());
  }

  // @Test
  // public void TestManual() {

  // }
}
