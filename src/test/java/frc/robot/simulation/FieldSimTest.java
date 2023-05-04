package frc.robot.simulation;

import static frc.robot.utils.ChargedUpNodeMask.updateNodeMask;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

import org.junit.jupiter.api.*;
import utils.TestUtils;

public class FieldSimTest {
  protected RobotContainer m_robotContainer;
  protected SwerveDrive m_swerveDrive;
  protected Controls m_controls;
  protected FieldSim m_fieldSim;

  @BeforeEach
  // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_robotContainer = new RobotContainer();
    m_swerveDrive = m_robotContainer.getSwerveDrive();
    m_controls = m_robotContainer.getControls();
    m_fieldSim = m_robotContainer.getFieldSim();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  // this method will run after each test
  void shutdown() throws Exception {
    m_robotContainer.close();
  }

  @Test
  public void testRedAllianceRedNodes() {
    TestUtils.setPrivateField(m_controls, "allianceColor", DriverStation.Alliance.Red);
    m_swerveDrive.setOdometry(new Pose2d(SimConstants.fieldLength, 0, Rotation2d.fromDegrees(0)));

    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.LOW);
    assertEquals(m_fieldSim.getValidNodes().size(), 9);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() > SimConstants.fieldLength / 2);
    }
    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.MID_CONE);
    assertEquals(m_fieldSim.getValidNodes().size(), 6);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() > SimConstants.fieldLength / 2);
    }
    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.MID_CUBE);
    assertEquals(m_fieldSim.getValidNodes().size(), 3);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() > SimConstants.fieldLength / 2);
    }
  }

  @Test
  public void testRedAllianceBlueCoopertitionNodes() {
    TestUtils.setPrivateField(m_controls, "allianceColor", DriverStation.Alliance.Red);
    m_swerveDrive.setOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.LOW);
    assertEquals(m_fieldSim.getValidNodes().size(), 3);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() < SimConstants.fieldLength / 2);
    }
    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.MID_CONE);
    assertEquals(m_fieldSim.getValidNodes().size(), 2);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() < SimConstants.fieldLength / 2);
    }
    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.MID_CUBE);
    assertEquals(m_fieldSim.getValidNodes().size(), 1);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() < SimConstants.fieldLength / 2);
    }
  }

  @Test
  public void testBlueAllianceBlueNodes() {
    TestUtils.setPrivateField(m_controls, "allianceColor", DriverStation.Alliance.Blue);
    m_swerveDrive.setOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.LOW);
    assertEquals(m_fieldSim.getValidNodes().size(), 9);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() < SimConstants.fieldLength / 2);
    }

    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.MID_CONE);
    assertEquals(m_fieldSim.getValidNodes().size(), 6);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() < SimConstants.fieldLength / 2);
    }

    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.MID_CUBE);
    assertEquals(m_fieldSim.getValidNodes().size(), 3);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() < SimConstants.fieldLength / 2);
    }
  }

  @Test
  public void testBlueAllianceRedCoopertitionNodes() {
    TestUtils.setPrivateField(m_controls, "allianceColor", DriverStation.Alliance.Blue);
    m_swerveDrive.setOdometry(new Pose2d(SimConstants.fieldLength, 0, Rotation2d.fromDegrees(0)));

    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.LOW);
    assertEquals(m_fieldSim.getValidNodes().size(), 3);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() > SimConstants.fieldLength / 2);
    }
    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.MID_CONE);
    assertEquals(m_fieldSim.getValidNodes().size(), 2);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() > SimConstants.fieldLength / 2);
    }
    updateNodeMask(m_swerveDrive.getPoseMeters(), SCORING_STATE.MID_CUBE);
    assertEquals(m_fieldSim.getValidNodes().size(), 1);
    for (var node : m_fieldSim.getValidNodes()) {
      assertTrue(node.getX() > SimConstants.fieldLength / 2);
    }
  }
}
