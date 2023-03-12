// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.simulation.SimConstants.Grids;
import frc.robot.subsystems.*;
import frc.robot.utils.ModuleMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class FieldSim extends SubsystemBase {
  private final SwerveDrive m_swerveDrive;
  private final Vision m_vision;
  private final Elevator m_elevator;
  private final Wrist m_wrist;
  private final Controls m_controls;

  private final Field2d m_field2d = new Field2d();

  private Pose2d robotPose;
  private Pose2d intakePose;

  /* Creates lists of the Pose2ds of each of the scoring nodes on the field, sorted into:
    - Cones and cubes
    - Alliance color
    - Low, mid and high positions
    - And coopertition grids
  */
  private ArrayList<Translation2d> validNodes = new ArrayList<>();

  private ArrayList<Translation2d> blueHybridNodes = new ArrayList<>();
  private ArrayList<Translation2d> blueMidConeNodes = new ArrayList<>();
  private ArrayList<Translation2d> blueMidCubeNodes = new ArrayList<>();
  private ArrayList<Translation2d> blueHighConeNodes = new ArrayList<>();
  private ArrayList<Translation2d> blueHighCubeNodes = new ArrayList<>();
  private ArrayList<Translation2d> blueCoopertitionNodes = new ArrayList<>();

  private ArrayList<Translation2d> blueNodes = new ArrayList<>();

  private ArrayList<Translation2d> redHybridNodes = new ArrayList<>();
  private ArrayList<Translation2d> redMidConeNodes = new ArrayList<>();
  private ArrayList<Translation2d> redMidCubeNodes = new ArrayList<>();
  private ArrayList<Translation2d> redHighConeNodes = new ArrayList<>();
  private ArrayList<Translation2d> redHighCubeNodes = new ArrayList<>();
  private ArrayList<Translation2d> redCoopertitionNodes = new ArrayList<>();

  private ArrayList<Translation2d> redNodes = new ArrayList<>();

  private ArrayList<Translation2d> ignoredNodes = new ArrayList<>();

  private DriverStation.Alliance m_currentAlliance = Alliance.Red;
  SendableChooser<SCORING_STATE> scoringStateChooser = new SendableChooser<>();
  private final boolean testScoringState = true;

  public FieldSim(
      SwerveDrive swerveDrive, Vision vision, Elevator elevator, Wrist wrist, Controls controls) {
    m_swerveDrive = swerveDrive;
    m_vision = vision;
    m_elevator = elevator;
    m_wrist = wrist;
    m_controls = controls;

    initSim();
  }

  public void initSim() {
    initializeScoringNodes();

    if (RobotBase.isSimulation()) {
      scoringStateChooser.setDefaultOption(SCORING_STATE.STOWED.toString(), SCORING_STATE.STOWED);
      for (int i = 1; i < SCORING_STATE.values().length; i++) {
        scoringStateChooser.addOption(
            SCORING_STATE.values()[i].toString(), SCORING_STATE.values()[i]);
      }
      SmartDashboard.putData("Test Scoring State", scoringStateChooser);
    }
  }

  public Field2d getField2d() {
    return m_field2d;
  }

  private void initializeScoringNodes() {
    // Split nodes into separate lists to make it easier to filter
    blueHybridNodes = new ArrayList<>(Arrays.asList(Grids.lowTranslations));
    redHybridNodes =
            blueHybridNodes.stream().map(SimConstants::allianceFlip).collect(Collectors.toCollection(ArrayList::new));

    for (int i = 0; i < Grids.nodeRowCount; i++) {
      boolean isCube = i == 1 || i == 4 || i == 7;

      // Generate Mid Nodes
      if (isCube) {
        blueMidCubeNodes.add(Grids.midTranslations[i]);
        redMidCubeNodes.add(SimConstants.allianceFlip(Grids.midTranslations[i]));
      } else {
        blueMidConeNodes.add(Grids.midTranslations[i]);
        redMidConeNodes.add(SimConstants.allianceFlip(Grids.midTranslations[i]));
      }

      // Generate High Nodes
      if (isCube) {
        blueHighCubeNodes.add(Grids.highTranslations[i]);
        redHighCubeNodes.add(SimConstants.allianceFlip(Grids.highTranslations[i]));
      } else {
        blueHighConeNodes.add(Grids.highTranslations[i]);
        redHighConeNodes.add(SimConstants.allianceFlip(Grids.highTranslations[i]));
      }
    }

    // Mark Cooperatition Nodes separately
    for (int i = 0; i < 3; i++) {
      Translation2d[] gridRow = new Translation2d[3];
      switch (i) {
        case 0:
          gridRow = Grids.lowTranslations;
          break;
        case 1:
          gridRow = Grids.midTranslations;
          break;
        case 2:
          gridRow = Grids.highTranslations;
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

  public void setTrajectory(List<PathPlannerTrajectory> trajectories) {
    List<Pose2d> trajectoryPoses = new ArrayList<>();

    for (var trajectory : trajectories) {
      trajectoryPoses.addAll(
          trajectory.getStates().stream()
              .map(state -> state.poseMeters)
              .collect(Collectors.toList()));
    }

    m_field2d.getObject("trajectory").setPoses(trajectoryPoses);
  }

  public void setTrajectory(PathPlannerTrajectory trajectory) {
    m_field2d.getObject("trajectory").setTrajectory(trajectory);
  }

  public void resetRobotPose(Pose2d pose) {
    m_field2d.setRobotPose(pose);
  }

  private void updateRobotPoses() {
    robotPose = m_swerveDrive.getPoseMeters();
    m_field2d.setRobotPose(robotPose);
    m_field2d
        .getObject("lLocalizerTagPoses")
        .setPoses(m_vision.getTagPoses2d(CAMERA_SERVER.LEFT_LOCALIZER));
    m_field2d
        .getObject("lLocalizerPoses")
        .setPoses(m_vision.getRobotPoses2d(CAMERA_SERVER.LEFT_LOCALIZER));
    m_field2d
        .getObject("lLocalizerPose")
        .setPose(m_vision.getRobotPose2d(CAMERA_SERVER.LEFT_LOCALIZER));
    m_field2d
        .getObject("rLocalizerTagPoses")
        .setPoses(m_vision.getTagPoses2d(CAMERA_SERVER.RIGHT_LOCALIZER));
    m_field2d
        .getObject("rLocalizerPoses")
        .setPoses(m_vision.getRobotPoses2d(CAMERA_SERVER.RIGHT_LOCALIZER));
    m_field2d
        .getObject("rLocalizerPose")
        .setPose(m_vision.getRobotPose2d(CAMERA_SERVER.RIGHT_LOCALIZER));

    m_field2d
        .getObject("fLocalizerPose")
        .setPose(m_vision.getRobotPose2d(CAMERA_SERVER.FUSED_LOCALIZER));

    intakePose =
        m_swerveDrive
            .getPoseMeters()
            .transformBy(
                new Transform2d(
                    m_elevator
                        .getElevatorField2dTranslation()
                        .plus(m_wrist.getHorizontalTranslation()),
                    m_swerveDrive.getHeadingRotation2d()));
    m_field2d.getObject("Intake Pose").setPose(intakePose);

    m_field2d
        .getObject("Grid Node")
        .setPoses(
            validNodes.stream()
                .map(t -> new Pose2d(t, Rotation2d.fromDegrees(0)))
                .collect(Collectors.toList()));

//    var testNodes = Arrays.stream(Grids.lowTranslations).map(t -> new Pose2d(t, Rotation2d.fromDegrees(0))).collect(Collectors.toCollection(ArrayList::new));
//    testNodes.replaceAll(SimConstants::allianceFlip);
//
//    m_field2d.getObject("TestNodes").setPoses(testNodes);

    if (RobotBase.isSimulation()) {
      m_field2d
          .getObject("Swerve Modules")
          .setPoses(ModuleMap.orderedValues(m_swerveDrive.getModulePoses(), new Pose2d[0]));
    }
  }

  /*
   * Priority list:
   * 1 - Node is on our alliance
   * 2 - Node takes our current game piece
   * 3 - Node is on the same level as our elevator
   * 4 - Node is closest to our robot
   */
  public void updateValidNodes(SCORING_STATE scoringState) {
//    if (RobotBase.isSimulation() && testScoringState) {
//      scoringState = scoringStateChooser.getSelected();
//    }

    ArrayList<Translation2d> filteredNodes;
    boolean filterRed;
    if (m_currentAlliance == Alliance.Red)
      if (m_swerveDrive.getPoseMeters().getX() > SimConstants.fieldLength / 2.0) {
        filteredNodes = (ArrayList<Translation2d>) redNodes.clone();
        filterRed = true;
      } else {
        filteredNodes = (ArrayList<Translation2d>) blueCoopertitionNodes.clone();
        filterRed = false;
      }
    else {
      if (m_swerveDrive.getPoseMeters().getX() < SimConstants.fieldLength / 2.0) {
        filteredNodes = (ArrayList<Translation2d>) blueNodes.clone();
        filterRed = false;
      } else {
        filteredNodes = (ArrayList<Translation2d>) redCoopertitionNodes.clone();
        filterRed = true;
      }
    }

    if (scoringState == SCORING_STATE.LOW || scoringState == SCORING_STATE.LOW_REVERSE) {
      filteredNodes.retainAll(filterRed? redHybridNodes : blueHybridNodes);
    } else if (scoringState == SCORING_STATE.MID_CONE) {
      filteredNodes.retainAll(filterRed? redMidConeNodes : blueMidConeNodes);
    } else if (scoringState == SCORING_STATE.MID_CUBE) {
      filteredNodes.retainAll(filterRed? redMidCubeNodes : blueMidCubeNodes);
    } else if (scoringState == SCORING_STATE.HIGH_CONE) {
      filteredNodes.retainAll(filterRed? redHighConeNodes : blueHighConeNodes);
    } else if (scoringState == SCORING_STATE.HIGH_CUBE) {
      filteredNodes.retainAll(filterRed? redHighCubeNodes : blueHighCubeNodes);
    }

    // Remove all nodes that are ignored (e.g. scored)
    filteredNodes.removeAll(ignoredNodes);

    validNodes = filteredNodes;
  }

  public ArrayList<Translation2d> getValidNodes() {
    return validNodes;
  }

  public Pose2d getTargetNode() {
    List<Pose2d> possibleNodes =
        getValidNodes().stream()
            .map(t -> new Pose2d(t, Rotation2d.fromDegrees(0)))
            .collect(Collectors.toList());
    // Only works on WPILIB version 2023.3.2 and above
    if (possibleNodes.isEmpty()) return new Pose2d(-1, -1, Rotation2d.fromDegrees(0));
    else return robotPose.nearest(possibleNodes);
  }

  // True for left, false for right
  public Pose2d getAdjacentNode(Pose2d node, ArrayList<Translation2d> possibleNodes, boolean left) {
    int nodeIndex = validNodes.indexOf(node.getTranslation());
    int newIndex = MathUtil.clamp(nodeIndex + (left ? -1 : 1), 0, possibleNodes.size());

    return new Pose2d(possibleNodes.get(newIndex), Rotation2d.fromDegrees(0));
  }

  @Override
  public void periodic() {
    if(RobotBase.isSimulation() || (RobotBase.isReal() && DriverStation.isDisabled())) {
      m_currentAlliance = Controls.getAllianceColor();
    }
    updateRobotPoses();

    if (RobotBase.isSimulation()) simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {}
}
