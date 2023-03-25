package frc.robot.commands.auto;

import static org.junit.jupiter.api.Assertions.*;
import static utils.TestUtils.getPrivateObject;
import static utils.TestUtils.setPrivateField;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AUTO;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.subsystems.SwerveDrive;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class AutoBalanceTest {
  protected RobotContainer m_robotContainer;
  protected SwerveDrive m_swerveDrive;

  @BeforeEach
  // this method will run before each test. We Initialize the RobotContainer and get all subsystems
  // from it for our tests
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_robotContainer = new RobotContainer();
    var subsystems = getPrivateObject(CommandScheduler.getInstance(), "m_subsystems");
    //    m_swerveDrive = (LinkedHashMap<Subsystem, Command>) subsystems;
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
  public void TestAutoBalanceTimeout() {
    var cmd = new AutoBalance(m_swerveDrive);

    // Override values for testing
    setPrivateField(m_swerveDrive, "m_simOverride", true);
    setPrivateField(m_swerveDrive, "m_rollOffset", 0);
    AUTO.kAutoBalanceTimeout = AUTO.kAutoBalanceTimeout / 100.0;

    Timer m_timer = new Timer();
    m_timer.reset();
    m_timer.start();
    cmd.initialize();

    setPrivateField(m_swerveDrive, "m_simRoll", -5);
    while(m_timer.get() < AUTO.kAutoBalanceTimeout * 4) {
      // Manually run the command and check if it finishes because CommandScheduler doesn't work properly in test
      cmd.execute();
      if(m_timer.get() < AUTO.kAutoBalanceTimeout) {
        assertFalse(cmd.isFinished());
      } else if (m_timer.get() < AUTO.kAutoBalanceTimeout + AUTO.kAutoBalanceTimeout * 0.5) {
        // Test that AutoBalance doesn't immediately return true once in tolerance
        setPrivateField(m_swerveDrive, "m_simRoll", -1);
        assertFalse(cmd.isFinished());
      } else if (m_timer.get() < AUTO.kAutoBalanceTimeout * 2.25) {
        // Test that AutoBalance doesn't return true if moved outside of tolerance
        setPrivateField(m_swerveDrive, "m_simRoll", -3);
        assertFalse(cmd.isFinished());
      } else if (m_timer.get() < AUTO.kAutoBalanceTimeout * 3.25) {
        setPrivateField(m_swerveDrive, "m_simRoll", -1);
        assertFalse(cmd.isFinished());
      } else if (m_timer.get() > AUTO.kAutoBalanceTimeout * 3.25 + 0.01) {
        // Test that AutoBalance only returns true if within tolerance greater than 2 seconds
        assertTrue(cmd.isFinished());
      }
    }
  }
}
