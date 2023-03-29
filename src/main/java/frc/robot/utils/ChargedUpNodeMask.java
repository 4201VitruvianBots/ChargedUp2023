package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.Controls;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/** Step 1: Define all nodes in an ArrayList Step 2: Use bit masks Step 3: ??? Step 4: Profit */
public class ChargedUpNodeMask {
  private static Map<Integer, Translation2d> allNodes = new HashMap<>();

  /** Node Definitions */
  // Starting with Blue Nodes, numbering starts at 0 with the rightmost LOW_HYBRID node, closest to
  // the Red Substations
  // and 26 is the leftmost HIGH_CONE node. 27 is the Blue single substation, 28 is the left Blue
  // double substation and
  // 29 is the right Blue double substation.
  // Red Node numbering is mirrored from blue nodes given the same pattern from above.
  private static final long blueNodes = 0b0000_0111_1111_1111_1111_1111_1111_1111L;

  private static final long blueHybridNodes = 0b0000_0000_0000_0000_0000_0001_1111_1111L;
  private static final long blueMidConeNodes = 0b0000_0000_0000_0010_1101_1010_0000_0000L;
  private static final long blueMidCubeNodes = 0b0000_0000_0000_0001_0010_0100_0000_0000L;
  private static final long blueHighConeNodes = 0b0000_0101_1011_0100_0000_0000_0000_0000L;
  private static final long blueHighCubeNodes = 0b0000_0010_0100_1000_0000_0000_0000_0000L;
  private static final long redCoopertitionNodes = 0b0000_0000_1110_0000_0111_0000_0011_1000L;

  private static final long redNodes = blueNodes << 32;
  private static final long redHybridNodes = blueHybridNodes << 32;
  private static final long redMidConeNodes = blueMidConeNodes << 32;
  private static final long redMidCubeNodes = blueMidCubeNodes << 32;
  private static final long redHighConeNodes = blueHighConeNodes << 32;
  private static final long redHighCubeNodes = blueHighCubeNodes << 32;
  private static final long blueCoopertitionNodes = redCoopertitionNodes << 32;

  private static final long blueConeNodes = blueHybridNodes | blueMidConeNodes | blueHighConeNodes;
  private static final long blueCubeNodes = blueHybridNodes | blueMidCubeNodes | blueHighCubeNodes;
  private static final long redConeNodes = blueConeNodes << 32;
  private static final long redCubeNodes = blueCubeNodes << 32;
  private static final long coneNodes = blueConeNodes | redConeNodes;
  private static final long cubeNodes = blueCubeNodes | redCubeNodes;

  private static long ignoredNodes = 0;
  private static long validNodeMask = 0;

  private static ArrayList<Translation2d> validNodes = new ArrayList<>();

  public static void initializeNodes() {
    // Split nodes into separate lists to make it easier to filter
    for (int i = 0; i < SimConstants.Grids.lowTranslations.length; i++) {
      allNodes.put(i, SimConstants.Grids.lowTranslations[i]);
      allNodes.put(i + 32, SimConstants.allianceFlip(SimConstants.Grids.lowTranslations[i]));
    }

    for (int i = 0; i < SimConstants.Grids.midTranslations.length; i++) {
      allNodes.put(i + 9, SimConstants.Grids.midTranslations[i]);
      allNodes.put(i + 9 + 32, SimConstants.allianceFlip(SimConstants.Grids.midTranslations[i]));
    }

    for (int i = 0; i < SimConstants.Grids.highTranslations.length; i++) {
      allNodes.put(i + 18, SimConstants.Grids.highTranslations[i]);
      allNodes.put(i + 18 + 32, SimConstants.allianceFlip(SimConstants.Grids.highTranslations[i]));
    }

    // Padding
    allNodes.put(27, new Translation2d());
    allNodes.put(28, new Translation2d());
    allNodes.put(29, new Translation2d());
    allNodes.put(30, new Translation2d());
    allNodes.put(31, new Translation2d());
    allNodes.put(27 + 32, SimConstants.allianceFlip(new Translation2d()));
    allNodes.put(28 + 32, SimConstants.allianceFlip(new Translation2d()));
    allNodes.put(29 + 32, SimConstants.allianceFlip(new Translation2d()));
    allNodes.put(30 + 32, SimConstants.allianceFlip(new Translation2d()));
    allNodes.put(31 + 32, SimConstants.allianceFlip(new Translation2d()));
  }

  public static void addIgnoredNode(int nodeIndex) {
    ignoredNodes = ignoredNodes | 1L << nodeIndex;
  }

  public static void removeIgnoredNode(int nodeIndex) {
    ignoredNodes = ignoredNodes & ~(1L << nodeIndex);
  }

  public static void updateValidNodes(Pose2d robotPose, SCORING_STATE scoringState) {
    validNodeMask = 0xFFFF_FFFF_FFFF_FFFFL;
    boolean usingRedNodes;
    if (Controls.getAllianceColor() == DriverStation.Alliance.Red) {
      if (robotPose.getX() > SimConstants.fieldLength / 2) {
        validNodeMask = validNodeMask & redNodes;
        usingRedNodes = true;
      } else {
        validNodeMask = validNodeMask & blueCoopertitionNodes;
        usingRedNodes = false;
      }
    } else {
      if (robotPose.getX() < SimConstants.fieldLength / 2) {
        validNodeMask = validNodeMask & blueNodes;
        usingRedNodes = false;
      } else {
        validNodeMask = validNodeMask & redCoopertitionNodes;
        usingRedNodes = true;
      }
    }

    if (scoringState == SCORING_STATE.LOW || scoringState == SCORING_STATE.LOW_REVERSE) {
      validNodeMask = validNodeMask & (usingRedNodes ? redHybridNodes : blueHybridNodes);
    } else if (scoringState == SCORING_STATE.MID_CONE) {
      validNodeMask = validNodeMask & (usingRedNodes ? redMidConeNodes : blueMidConeNodes);
    } else if (scoringState == SCORING_STATE.MID_CUBE) {
      validNodeMask = validNodeMask & (usingRedNodes ? redMidCubeNodes : blueMidCubeNodes);
    } else if (scoringState == SCORING_STATE.HIGH_CONE) {
      validNodeMask = validNodeMask & (usingRedNodes ? redHighConeNodes : blueHighConeNodes);
    } else if (scoringState == SCORING_STATE.HIGH_CUBE) {
      validNodeMask = validNodeMask & (usingRedNodes ? redHighCubeNodes : blueHighCubeNodes);
    }

    validNodeMask = validNodeMask & ~(ignoredNodes);
  }

  public static ArrayList<Translation2d> getValidNodes() {
    validNodes.clear();

    for (int i = 0; i < allNodes.size(); i++) {
      long mask = 1L << i;
      long validNode = validNodeMask & mask;
      if (validNode != 0) {
        validNodes.add(allNodes.get(i));
      }
    }

    return validNodes;
  }

  public static Pose2d getTargetNode(Pose2d robotPose) {
    var validNodes = getValidNodes();
    if (validNodes.size() == 0) return new Pose2d();
    else
      return new Pose2d(robotPose.getTranslation().nearest(validNodes), Rotation2d.fromDegrees(0));
  }

  // TODO: change second argument to an enum
  public static Pose2d getTargetNode(Pose2d robotPose, int gamePieceType) {
    validNodeMask = validNodeMask & ~(gamePieceType == 0 ? coneNodes : cubeNodes);
    var validNodes = getValidNodes();
    if (validNodes.size() == 0) return new Pose2d();
    else
      return new Pose2d(robotPose.getTranslation().nearest(validNodes), Rotation2d.fromDegrees(0));
  }
}
