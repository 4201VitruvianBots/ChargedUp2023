package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CommandTestBase;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.statehandler.SetSetpoint;
import frc.robot.subsystems.elevator.Elevator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

public class IntakeTest extends CommandTestBase {
  protected RobotContainer m_robotContainer;
  protected Elevator m_elevator;
  protected Wrist m_wrist;
  protected Intake m_intake;
  protected StateHandler m_stateHandler;

  @BeforeEach
  // this method will run before each test. We Initialize the RobotContainer and get all subsystems
  // from it for our tests
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_robotContainer = new RobotContainer();
    m_elevator = m_robotContainer.getElevator();
    m_wrist = m_robotContainer.getWrist();
    m_intake = m_robotContainer.getIntake();
    m_stateHandler = m_robotContainer.getStateHandler();
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
  @Disabled("WIP")
  @Test
  public void TestAutoStow() {
    var setSetpointCommand =
        new SetSetpoint(m_stateHandler, m_elevator, m_wrist, SETPOINT.INTAKING_LOW_CUBE);
    var intakeCommand = new SetIntakeState(m_intake, INTAKE_STATE.INTAKING_CUBE);

    CommandScheduler.getInstance().schedule(setSetpointCommand);
    CommandScheduler.getInstance().schedule(intakeCommand);

    // Go To intake position
    do {
      CommandScheduler.getInstance().run();
    } while (!(Math.abs(
                m_elevator.getHeightMeters()
                    - SETPOINT.INTAKING_LOW_CUBE.getElevatorSetpointMeters())
            < 0.01)
        || !(Math.abs(
                m_wrist.getPositionRadians() - SETPOINT.INTAKING_LOW_CUBE.getWristSetpointRadians())
            < Units.degreesToRadians(1)));

    Timer m_timer = new Timer();
    m_timer.reset();
    m_timer.start();
    while (m_timer.get() < 1) {
      CommandScheduler.getInstance().run();
      assertSame(m_stateHandler.getDesiredSetpoint(), SETPOINT.INTAKING_LOW_CUBE);
      assertSame(m_intake.getIntakeState(), INTAKE_STATE.INTAKING_CUBE);
    }
    m_intake.getIntakeMotor().getSimCollection().setIntegratedSensorVelocity(0);
    CommandScheduler.getInstance().run();

    assertSame(m_stateHandler.getDesiredSetpoint(), SETPOINT.STOWED);
    assertSame(m_intake.getIntakeState(), INTAKE_STATE.HOLDING_CUBE);
  }
}
