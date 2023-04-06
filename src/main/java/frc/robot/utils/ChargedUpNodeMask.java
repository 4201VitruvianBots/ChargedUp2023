package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.Controls;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/** Step 1: Define all nodes in a Map Step 2: Use bit masks Step 3: ??? Step 4: Profit */
public class ChargedUpNodeMask {
  private static final Map<Integer, Translation2d> blueNodes = new HashMap<>();
  private static final Map<Integer, Translation2d> redNodes = new HashMap<>();
  private static Map<Integer, Translation2d> currentNodes = redNodes;

  /** Node Definitions */
  // Starting with Blue Nodes, numbering starts at 0 with the rightmost LOW_HYBRID node, closest to
  // the Red Substations and 26 is the leftmost HIGH_CONE node. 27 is the Blue single substation,
  // 28 is the left Blue double substation and 29 is the right Blue double substation.
  // Red Node numbering is mirrored from blue nodes given the same pattern from above.
  private static final int gridNodes = 0b0000_0111_1111_1111_1111_1111_1111_1111;

  private static final int hybridNodes = 0b0000_0000_0000_0000_0000_0001_1111_1111;
  private static final int midConeNodes = 0b0000_0000_0000_0010_1101_1010_0000_0000;
  private static final int midCubeNodes = 0b0000_0000_0000_0001_0010_0100_0000_0000;
  private static final int highConeNodes = 0b0000_0101_1011_0100_0000_0000_0000_0000;
  private static final int highCubeNodes = 0b0000_0010_0100_1000_0000_0000_0000_0000;

  private static final int coopertitionNodes = 0b0000_0000_1110_0000_0111_0000_0011_1000;

  private static final int coneNodes = hybridNodes | midConeNodes | highConeNodes;
  private static final int cubeNodes = hybridNodes | midCubeNodes | highCubeNodes;

  private static int ignoredRedNodes = 0;
  private static int ignoredBlueNodes = 0;
  private static int validNodeMask = 0;

  private static final ArrayList<Translation2d> validNodes = new ArrayList<>();

  public static void initializeNodeMaps() {
    // Split nodes into separate lists to make it easier to filter
    for (int i = 0; i < SimConstants.Grids.lowTranslations.length; i++) {
      blueNodes.put(i, SimConstants.Grids.lowTranslations[i]);
      redNodes.put(i, SimConstants.allianceFlip(SimConstants.Grids.lowTranslations[i]));
    }

    for (int i = 0; i < SimConstants.Grids.midTranslations.length; i++) {
      blueNodes.put(i + 9, SimConstants.Grids.midTranslations[i]);
      redNodes.put(i + 9, SimConstants.allianceFlip(SimConstants.Grids.midTranslations[i]));
    }

    for (int i = 0; i < SimConstants.Grids.highTranslations.length; i++) {
      blueNodes.put(i + 18, SimConstants.Grids.highTranslations[i]);
      redNodes.put(i + 18, SimConstants.allianceFlip(SimConstants.Grids.highTranslations[i]));
    }

    // Padding
    //    blueNodes.put(27, new Translation2d());
    //    blueNodes.put(28, new Translation2d());
    //    blueNodes.put(29, new Translation2d());
    //    blueNodes.put(30, new Translation2d());
    //    blueNodes.put(31, new Translation2d());
    //    redNodes.put(27, SimConstants.allianceFlip(new Translation2d()));
    //    redNodes.put(28, SimConstants.allianceFlip(new Translation2d()));
    //    redNodes.put(29, SimConstants.allianceFlip(new Translation2d()));
    //    redNodes.put(30, SimConstants.allianceFlip(new Translation2d()));
    //    redNodes.put(31, SimConstants.allianceFlip(new Translation2d()));
  }

  public static void addIgnoredNode(int nodeIndex) {
    if (nodeIndex < 32) ignoredBlueNodes = ignoredBlueNodes | 1 << nodeIndex;
    else if (nodeIndex < 64) ignoredRedNodes = ignoredRedNodes | 1 << (nodeIndex - 32);
  }

  public static void removeIgnoredNode(int nodeIndex) {
    if (nodeIndex < 32) ignoredBlueNodes = ignoredBlueNodes & ~(1 << nodeIndex);
    else if (nodeIndex < 64) ignoredRedNodes = ignoredRedNodes & ~(1 << (nodeIndex - 32));
  }

  public static void updateNodeMask(Pose2d robotPose, SCORING_STATE scoringState) {
    validNodeMask = 0xFFFF_FFFF;

    if (Controls.getAllianceColor() == DriverStation.Alliance.Red) {
      if (robotPose.getX() > SimConstants.fieldLength / 2) {
        validNodeMask = gridNodes & ~ignoredRedNodes;
        currentNodes = redNodes;
      } else {
        validNodeMask = coopertitionNodes & ~ignoredBlueNodes;
        currentNodes = blueNodes;
      }
    } else {
      if (robotPose.getX() < SimConstants.fieldLength / 2) {
        validNodeMask = gridNodes & ~ignoredBlueNodes;
        currentNodes = blueNodes;
      } else {
        validNodeMask = coopertitionNodes & ~ignoredRedNodes;
        currentNodes = redNodes;
      }
    }

    if (scoringState == SCORING_STATE.LOW || scoringState == SCORING_STATE.LOW_REVERSE) {
      validNodeMask = validNodeMask & hybridNodes;
    } else if (scoringState == SCORING_STATE.MID_CONE) {
      validNodeMask = validNodeMask & midConeNodes;
    } else if (scoringState == SCORING_STATE.MID_CUBE) {
      validNodeMask = validNodeMask & midCubeNodes;
    } else if (scoringState == SCORING_STATE.HIGH_CONE) {
      validNodeMask = validNodeMask & highConeNodes;
    } else if (scoringState == SCORING_STATE.HIGH_CUBE) {
      validNodeMask = validNodeMask & highCubeNodes;
    }
  }

  public static ArrayList<Translation2d> getValidNodes() {
    validNodes.clear();

    for (int i = 0; i < 27; i++) {
      int mask = 1 << i;
      int validNode = validNodeMask & mask;
      if (validNode != 0) {
        validNodes.add(currentNodes.get(i));
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

  public static Pose2d getTargetNode(Pose2d robotPose, INTAKE_STATE intakeState) {
    if (intakeState == INTAKE_STATE.CONE) {
      validNodeMask = validNodeMask & ~coneNodes;
    }
    if (intakeState == INTAKE_STATE.CUBE) {
      validNodeMask = validNodeMask & ~cubeNodes;
    }
    var validNodes = getValidNodes();
    if (validNodes.size() == 0) return new Pose2d();
    else
      return new Pose2d(robotPose.getTranslation().nearest(validNodes), Rotation2d.fromDegrees(0));
  }
}
