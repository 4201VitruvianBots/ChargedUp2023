// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static frc.robot.utils.ChargedUpNodeMask.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.utils.ChargedUpNodeMask;
import frc.robot.utils.ModuleMap;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class FieldSim extends SubsystemBase implements AutoCloseable {
  private final SwerveDrive m_swerveDrive;
  private final Vision m_vision;
  private final Elevator m_elevator;
  private final Wrist m_wrist;
  private final StateHandler m_stateHandler;
  private final Controls m_controls;

  private final Field2d m_field2d = new Field2d();
  private List<PathPlannerTrajectory> m_displayedTrajectories = new ArrayList<>();
  private DriverStation.Alliance m_displayedAlliance = DriverStation.Alliance.Blue;

  private ArrayList<Pose2d> m_displayedNodes = new ArrayList<>();
  private Pose2d m_highlightedNode = new Pose2d(0, 0, new Rotation2d(0));

  private Pose2d robotPose = new Pose2d(0, 0, new Rotation2d(0));
  private Pose2d intakePose;

  public FieldSim(
      SwerveDrive swerveDrive,
      Vision vision,
      Elevator elevator,
      Wrist wrist,
      StateHandler stateHandler,
      Controls controls) {
    m_swerveDrive = swerveDrive;
    m_vision = vision;
    m_elevator = elevator;
    m_wrist = wrist;
    m_stateHandler = stateHandler;
    m_controls = controls;

    initSim();
  }

  public void initSim() {
    initializeScoringNodes();
  }

  public Field2d getField2d() {
    return m_field2d;
  }

  /**
   * Initialize arrays with all the scoring positions on the field based on alliance color, game
   * piece type, and if it is a coopertition node. Ideally, this is a pre-processing step that we
   * only need to do once to improve robot code performance by avoiding unnecessary repeated calls.
   */
  private void initializeScoringNodes() {
    initializeNodeMaps();
  }

  public void setTrajectory(List<PathPlannerTrajectory> trajectories) {
    if (!m_displayedTrajectories.equals(trajectories)
        || !m_displayedAlliance.equals(Controls.getAllianceColor())) {
      List<Pose2d> trajectoryPoses = new ArrayList<>();

      if (Controls.getAllianceColor() == DriverStation.Alliance.Red)
        for (var trajectory : trajectories) {
          trajectoryPoses.addAll(
              trajectory.getStates().stream()
                  .map(state -> SimConstants.pathPlannerFlip(state.poseMeters))
                  .collect(Collectors.toList()));
        }
      else {
        for (var trajectory : trajectories) {
          trajectoryPoses.addAll(
              trajectory.getStates().stream()
                  .map(state -> state.poseMeters)
                  .collect(Collectors.toList()));
        }
      }

      m_field2d.getObject("trajectory").setPoses(trajectoryPoses);
      m_displayedTrajectories = trajectories;
      m_displayedAlliance = Controls.getAllianceColor();
    }
  }

  public void resetRobotPose(Pose2d pose) {
    m_field2d
        .getObject("Swerve Modules")
        .setPoses(ModuleMap.orderedValues(m_swerveDrive.getModulePoses(), new Pose2d[0]));
    m_field2d.setRobotPose(pose);
  }

  public ArrayList<Translation2d> getValidNodes() {
    return ChargedUpNodeMask.getValidNodes();
  }

  /**
   * Based on the current robot's state and the list of valid nodes, return the nearest node for
   * scoring
   *
   * @return {@link Pose2d} Nearest Pose to robot
   */
  public Pose2d getTargetNode() {
    return ChargedUpNodeMask.getTargetNode(m_swerveDrive.getPoseMeters());
  }

  // TODO: Unit Test this with new node mask implementation
  // True for left, false for right
  public Pose2d getAdjacentNode(Pose2d node, ArrayList<Translation2d> possibleNodes, boolean left) {
    return new Pose2d();
  }

  public void setDisplayedNodes(ArrayList<Pose2d> displayedNodes, Pose2d highlightedNode) {
    m_displayedNodes = displayedNodes;
    m_highlightedNode = highlightedNode;
  }

  /**
   * Return a list of valid nodes for scoring based on the following priorities: [1] - Node is on
   * our alliance or coopertition grid. [2] - Node takes our current game piece. [3] - Node is on
   * the same level as our elevator. [4] - Node is closest to our robot
   */
  public void updateValidNodes() {
    updateNodeMask(m_swerveDrive.getPoseMeters(), m_stateHandler.getScoringState());
  }

  private void updateRobotPoses() {
    robotPose = m_swerveDrive.getPoseMeters();
    m_field2d.setRobotPose(robotPose);

    if (RobotBase.isSimulation()) {
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
    }

    m_field2d
        .getObject("fLocalizerPose")
        .setPose(m_vision.getRobotPose2d(CAMERA_SERVER.FUSED_LOCALIZER));

    intakePose =
        m_swerveDrive
            .getPoseMeters()
            .transformBy(
                new Transform2d(
                    m_elevator.getField2dTranslation().plus(m_wrist.getHorizontalTranslation()),
                    m_swerveDrive.getHeadingRotation2d()));
    m_field2d.getObject("Intake Pose").setPose(intakePose);

    m_field2d
        .getObject("Grid Node")
        .setPoses(
            getValidNodes().stream()
                .map(t -> new Pose2d(t, Rotation2d.fromDegrees(0)))
                .collect(Collectors.toList()));

    if (RobotBase.isSimulation()) {
      m_field2d
          .getObject("Swerve Modules")
          .setPoses(ModuleMap.orderedValues(m_swerveDrive.getModulePoses(), new Pose2d[0]));

      if (getTargetNode().equals(new Pose2d())) {
        m_field2d.getObject("RobotToNodeF").setPoses(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
        m_field2d.getObject("RobotToNodeT").setPoses(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
      } else {
        if (m_stateHandler.canScore()) {
          m_field2d
              .getObject("RobotToNodeT")
              .setPoses(m_swerveDrive.getPoseMeters(), getTargetNode());
          m_field2d
              .getObject("RobotToNodeF")
              .setPoses(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
        } else {
          m_field2d
              .getObject("RobotToNodeF")
              .setPoses(m_swerveDrive.getPoseMeters(), getTargetNode());
          m_field2d
              .getObject("RobotToNodeT")
              .setPoses(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
        }
      }
    }
  }

  @Override
  public void periodic() {
    updateRobotPoses();

    try {
      SmartDashboard.putData("Field2d", m_field2d);
    } catch (NullPointerException e) {
      //      e.printStackTrace();
    }
  }

  @Override
  public void simulationPeriodic() {}

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
