package frc.robot.simulation;

import static frc.robot.utils.ChargedUpNodeMask.updateNodeMask;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.RobotContainer;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

@Disabled("Works, but too many tests breaks builds")
public class NodeFilteringTest {
  protected RobotContainer m_robotContainer;
  protected SwerveDrive m_swerveDrive;
  protected FieldSim m_fieldSim;

  @BeforeEach
  // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_robotContainer = new RobotContainer();
    m_swerveDrive = m_robotContainer.getSwerveDrive();
    m_fieldSim = m_robotContainer.getFieldSim();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  // this method will run after each test
  void shutdown() throws Exception {
    m_robotContainer.close();
  }

  @Test
  public void updateValidNodesTiming() {
    Timer m_timer = new Timer();

    SCORING_STATE[] states = {
      SCORING_STATE.LOW,
      SCORING_STATE.MID_CONE,
      SCORING_STATE.MID_CUBE,
      SCORING_STATE.HIGH_CONE,
      SCORING_STATE.HIGH_CUBE,
      SCORING_STATE.STOWED
    };
    double[] durations = new double[states.length];
    double totalTime = 0;

    for (int i = 0; i < states.length; i++) {
      m_timer.reset();
      m_timer.start();
      double m_timestamp = m_timer.get();
      updateNodeMask(m_swerveDrive.getPoseMeters(), states[i]);
      durations[i] = m_timer.get() - m_timestamp;
      totalTime += durations[i];
    }
    double average = totalTime / states.length;

    //    System.out.println("Avg. Duration: " + average * 1000.0 + "ms");
    assertTrue(average < 0.020);
  }
}
