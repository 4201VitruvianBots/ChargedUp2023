package frc.robot.commands.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static utils.TestUtils.getPrivateObject;
import static utils.TestUtils.setPrivateField;

import com.ctre.phoenix.platform.PlatformJNI;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.PigeonImuJNI;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.subsystems.Wrist;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.subsystems.SwerveDrive;
import utils.TestUtils;

import java.util.LinkedHashMap;

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
    m_swerveDrive = (LinkedHashMap<Subsystem, Command>) subsystems;
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
    var cmd = new WaitCommand(5);
//    var cmd = new AutoBalance(m_swerveDrive);
//    cmd.addRequirements(m_swerveDrive);
////    cmd.schedule();
//    CommandScheduler.getInstance().schedule(cmd);
//    CommandScheduler.getInstance().run();
//    m_swerveDrive.setDefaultCommand(new SetSwerveDrive(m_swerveDrive, ()-> 0, ()-> 0, ()-> 0));
    setPrivateField(m_swerveDrive, "m_simOverride", true);
    setPrivateField(m_swerveDrive, "m_simRoll", -1);


    Timer m_timer = new Timer();
    m_timer.reset();
    m_timer.start();
//    while(m_timer.get() < 3) {
    while(true) {
      CommandScheduler.getInstance().run();
      if(CommandScheduler.getInstance().isScheduled(cmd)) {
        System.out.println("TEST");
      }
      if(cmd.isScheduled()) {
        System.out.println("TEST");
      }

      if(m_swerveDrive.getCurrentCommand() != null) {
        if (m_timer.get() < 2.0) {
          assertEquals(m_swerveDrive.getCurrentCommand().getClass(), cmd.getClass());
        } else {
          assertNotEquals(m_swerveDrive.getCurrentCommand().getClass(), cmd.getClass());
        }
      } else {
        System.out.println("???");
      }
    }
  }
}
