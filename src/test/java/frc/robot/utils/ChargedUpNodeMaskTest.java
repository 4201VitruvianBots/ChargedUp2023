package frc.robot.utils;

import static frc.robot.utils.ChargedUpNodeMask.*;
import static frc.robot.utils.ChargedUpNodeMask.getValidNodes;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static utils.TestUtils.setPrivateField;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.JNIWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.RobotContainer;
import frc.robot.simulation.SimConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.stream.Collectors;

import frc.robot.subsystems.Controls;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ChargedUpNodeMaskTest {
  protected RobotContainer m_robotContainer;
  protected Controls m_controls;

  private static ArrayList<Pose2d> allNodes = new ArrayList<>();
  private static ArrayList<Translation2d> validNodes = new ArrayList<>();

  private static ArrayList<Translation2d> blueHybridNodes = new ArrayList<>();
  private static ArrayList<Translation2d> blueMidConeNodes = new ArrayList<>();
  private static ArrayList<Translation2d> blueMidCubeNodes = new ArrayList<>();
  private static ArrayList<Translation2d> blueHighConeNodes = new ArrayList<>();
  private static ArrayList<Translation2d> blueHighCubeNodes = new ArrayList<>();
  private static ArrayList<Translation2d> blueCoopertitionNodes = new ArrayList<>();

  private static ArrayList<Translation2d> blueNodes = new ArrayList<>();
  private static ArrayList<Translation2d> redHybridNodes = new ArrayList<>();
  private static ArrayList<Translation2d> redMidConeNodes = new ArrayList<>();
  private static ArrayList<Translation2d> redMidCubeNodes = new ArrayList<>();
  private static ArrayList<Translation2d> redHighConeNodes = new ArrayList<>();
  private static ArrayList<Translation2d> redHighCubeNodes = new ArrayList<>();
  private static ArrayList<Translation2d> redCoopertitionNodes = new ArrayList<>();

  private static ArrayList<Translation2d> redNodes = new ArrayList<>();

  private ArrayList<Translation2d> ignoredNodes = new ArrayList<>();

  @BeforeAll
  static void setupNodeArrays() {
    initializeNodes();

    // Split nodes into separate lists to make it easier to filter
    blueHybridNodes = new ArrayList<>(Arrays.asList(SimConstants.Grids.lowTranslations));
    redHybridNodes =
        blueHybridNodes.stream()
            .map(SimConstants::allianceFlip)
            .collect(Collectors.toCollection(ArrayList::new));

    for (int i = 0; i < SimConstants.Grids.nodeRowCount; i++) {
      boolean isCube = i == 1 || i == 4 || i == 7;

      // Generate Mid Nodes
      if (isCube) {
        blueMidCubeNodes.add(SimConstants.Grids.midTranslations[i]);
        redMidCubeNodes.add(SimConstants.allianceFlip(SimConstants.Grids.midTranslations[i]));
      } else {
        blueMidConeNodes.add(SimConstants.Grids.midTranslations[i]);
        redMidConeNodes.add(SimConstants.allianceFlip(SimConstants.Grids.midTranslations[i]));
      }

      // Generate High Nodes
      if (isCube) {
        blueHighCubeNodes.add(SimConstants.Grids.highTranslations[i]);
        redHighCubeNodes.add(SimConstants.allianceFlip(SimConstants.Grids.highTranslations[i]));
      } else {
        blueHighConeNodes.add(SimConstants.Grids.highTranslations[i]);
        redHighConeNodes.add(SimConstants.allianceFlip(SimConstants.Grids.highTranslations[i]));
      }
    }

    // Mark Cooperatition Nodes separately
    for (int i = 0; i < 3; i++) {
      Translation2d[] gridRow = new Translation2d[3];
      switch (i) {
        case 0:
          gridRow = SimConstants.Grids.lowTranslations;
          break;
        case 1:
          gridRow = SimConstants.Grids.midTranslations;
          break;
        case 2:
          gridRow = SimConstants.Grids.highTranslations;
          break;
      }
      for (int j = 3; j < 6; j++) {
        blueCoopertitionNodes.add(gridRow[j]);
        redCoopertitionNodes.add(SimConstants.allianceFlip(gridRow[j]));
      }
    }
    blueNodes.addAll(blueHybridNodes);
    blueNodes.addAll(blueMidConeNodes);
    blueNodes.addAll(blueMidCubeNodes);
    blueNodes.addAll(blueHighConeNodes);
    blueNodes.addAll(blueHighCubeNodes);

    redNodes.addAll(redHybridNodes);
    redNodes.addAll(redMidConeNodes);
    redNodes.addAll(redMidCubeNodes);
    redNodes.addAll(redHighConeNodes);
    redNodes.addAll(redHighCubeNodes);
  }

  @BeforeEach
  // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_robotContainer = new RobotContainer();
    m_controls = m_robotContainer.getControls();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  // this method will run after each test
  void shutdown() throws Exception {
    m_robotContainer.close();
  }

  @Test
  public void TestRedGetRedNodes() {
    setPrivateField(m_controls, "allianceColor", DriverStation.Alliance.Red);
    m_controls.updateAllianceColor();

    Pose2d robotPose = new Pose2d(14, 1, Rotation2d.fromDegrees(0));
    SCORING_STATE state;

    state = SCORING_STATE.STOWED;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redNodes));

    state = SCORING_STATE.LOW;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redHybridNodes));

    state = SCORING_STATE.MID_CONE;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redMidConeNodes));

    state = SCORING_STATE.MID_CUBE;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redMidCubeNodes));

    state = SCORING_STATE.HIGH_CONE;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redHighConeNodes));

    state = SCORING_STATE.HIGH_CUBE;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redHighCubeNodes));
  }

  @Test
  public void TestBlueGetBlueNodes() {
    setPrivateField(m_controls, "allianceColor", DriverStation.Alliance.Blue);

    Pose2d robotPose = new Pose2d();
    SCORING_STATE state;

    state = SCORING_STATE.STOWED;
    updateValidNodes(robotPose, state);
    var a = getValidNodes();
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueNodes));

    state = SCORING_STATE.LOW;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueHybridNodes));

    state = SCORING_STATE.MID_CONE;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueMidConeNodes));

    state = SCORING_STATE.MID_CUBE;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueMidCubeNodes));

    state = SCORING_STATE.HIGH_CONE;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueHighConeNodes));

    state = SCORING_STATE.HIGH_CUBE;
    updateValidNodes(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueHighCubeNodes));
  }
}
