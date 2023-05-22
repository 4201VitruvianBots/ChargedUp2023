package frc.robot.utils;

import static frc.robot.utils.ChargedUpNodeMask.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.RobotContainer;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.Controls;
import java.util.ArrayList;
import java.util.HashSet;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

public class ChargedUpNodeMaskTest {
  protected RobotContainer m_robotContainer;
  protected Controls m_controls;

  private static final ArrayList<Translation2d> blueNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> blueHybridNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> blueMidConeNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> blueMidCubeNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> blueHighConeNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> blueHighCubeNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> blueCoopertitionNodes = new ArrayList<>();

  private static final ArrayList<Translation2d> redNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> redHybridNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> redMidConeNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> redMidCubeNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> redHighConeNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> redHighCubeNodes = new ArrayList<>();
  private static final ArrayList<Translation2d> redCoopertitionNodes = new ArrayList<>();

  private static final ArrayList<Translation2d> ignoredNodes = new ArrayList<>();

  @BeforeAll
  static void setupNodeArrays() {
    initializeNodeMaps();

    // Split nodes into separate lists to make it easier to filter
    for (int i = 0; i < SimConstants.Grids.lowTranslations.length; i++) {
      blueHybridNodes.add(SimConstants.Grids.lowTranslations[i]);
      redHybridNodes.add(SimConstants.allianceFlip(SimConstants.Grids.lowTranslations[i]));
    }

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

    // Mark Coopertition Nodes separately
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
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    m_controls.periodic();

    Pose2d robotPose = new Pose2d(14, 1, Rotation2d.fromDegrees(0));
    SCORING_STATE state;

    state = SCORING_STATE.STOWED;
    updateNodeMask(robotPose, state);
    // Use HashSet to ignore list order
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redNodes));

    state = SCORING_STATE.LOW;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redHybridNodes));

    state = SCORING_STATE.MID_CONE;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redMidConeNodes));

    state = SCORING_STATE.MID_CUBE;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redMidCubeNodes));

    state = SCORING_STATE.HIGH_CONE;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redHighConeNodes));

    state = SCORING_STATE.HIGH_CUBE;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(redHighCubeNodes));
  }

  @Test
  @Disabled("For now")
  public void TestBlueGetBlueNodes() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    m_controls.periodic();

    Pose2d robotPose = new Pose2d();
    SCORING_STATE state;

    state = SCORING_STATE.STOWED;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueNodes));

    state = SCORING_STATE.LOW;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueHybridNodes));

    state = SCORING_STATE.MID_CONE;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueMidConeNodes));

    state = SCORING_STATE.MID_CUBE;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueMidCubeNodes));

    state = SCORING_STATE.HIGH_CONE;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueHighConeNodes));

    state = SCORING_STATE.HIGH_CUBE;
    updateNodeMask(robotPose, state);
    assertEquals(new HashSet<>(getValidNodes()), new HashSet<>(blueHighCubeNodes));
  }
}
