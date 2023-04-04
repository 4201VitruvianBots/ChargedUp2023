package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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

    Constants.SCORING_STATE[] states = {
      Constants.SCORING_STATE.LOW,
      Constants.SCORING_STATE.MID_CONE,
      Constants.SCORING_STATE.MID_CUBE,
      Constants.SCORING_STATE.HIGH_CONE,
      Constants.SCORING_STATE.HIGH_CUBE,
      Constants.SCORING_STATE.STOWED
    };
    double[] durations = new double[states.length];
    double totalTime = 0;

    for (int i = 0; i < states.length; i++) {
      m_timer.reset();
      m_timer.start();
      double m_timestamp = m_timer.get();
      m_fieldSim.updateValidNodes(states[i]);
      durations[i] = m_timer.get() - m_timestamp;
      totalTime += durations[i];
    }
    double average = totalTime / states.length;

    //    System.out.println("Avg. Duration: " + average * 1000.0 + "ms");
    assertTrue(average < 0.020);
  }
}
