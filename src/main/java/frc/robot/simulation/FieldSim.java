// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.simulation.SimConstants.Grids;
import frc.robot.subsystems.*;
import frc.robot.utils.ModuleMap;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class FieldSim extends SubsystemBase {
  private final SwerveDrive m_swerveDrive;
  private final Vision m_vision;
  private final Elevator m_elevator;
  private final Wrist m_wrist;
  private final Controls m_controls;

  private final Field2d m_field2d = new Field2d();

  private static Pose2d robotPose;
  private static Pose2d intakePose;

  /* Creates lists of the Pose2ds of each of the scoring nodes on the field, sorted into:
    - Cones and cubes
    - Alliance color
    - Low, mid and high positions
    - And coopertition grids
  */
  private static ArrayList<Pose2d> gridNodes = new ArrayList<>();

  private ArrayList<Pose2d> coneNodes = new ArrayList<>();
  private ArrayList<Pose2d> cubeNodes = new ArrayList<>();

  private ArrayList<Pose2d> blueNodes = new ArrayList<>();
  private ArrayList<Pose2d> redNodes = new ArrayList<>();

  private ArrayList<Pose2d> lowNodes = new ArrayList<>();
  private ArrayList<Pose2d> midNodes = new ArrayList<>();
  private ArrayList<Pose2d> highNodes = new ArrayList<>();

  private ArrayList<Pose2d> coopertitionNodes = new ArrayList<>();

  public FieldSim(
      SwerveDrive swerveDrive, Vision vision, Elevator elevator, Wrist wrist, Controls controls) {

    boolean cone = true;

    for (int i = 0; i < Grids.nodeRowCount; i++) {
      // Adds a new row of poses to the list using sim constants
      gridNodes.add(
          new Pose2d(
              Grids.outerX / 2 + Grids.lowX,
              Grids.nodeFirstY + (Grids.nodeSeparationY * i),
              new Rotation2d(0)));
      gridNodes.add(
          new Pose2d(
              Grids.outerX / 2 + Grids.midX,
              Grids.nodeFirstY + (Grids.nodeSeparationY * i),
              new Rotation2d(0)));
      gridNodes.add(
          new Pose2d(
              Grids.outerX / 2 + Grids.highX,
              Grids.nodeFirstY + (Grids.nodeSeparationY * i),
              new Rotation2d(0)));

      gridNodes.add(
          new Pose2d(
              SimConstants.fieldLength - (Grids.outerX / 2 + Grids.lowX),
              Grids.nodeFirstY + (Grids.nodeSeparationY * i),
              new Rotation2d(0)));
      gridNodes.add(
          new Pose2d(
              SimConstants.fieldLength - (Grids.outerX / 2 + Grids.midX),
              Grids.nodeFirstY + (Grids.nodeSeparationY * i),
              new Rotation2d(0)));
      gridNodes.add(
          new Pose2d(
              SimConstants.fieldLength - (Grids.outerX / 2 + Grids.highX),
              Grids.nodeFirstY + (Grids.nodeSeparationY * i),
              new Rotation2d(0)));

      // Think of i as our row, j as our column in this for loop
      for (int j = 0; j < 6; j++) {
        Pose2d currentNode = gridNodes.get(i * 6 + j);

        // Adds the poses to the cooperatition node if applicable
        if (i >= 3 && i <= 5) {
          coopertitionNodes.add(currentNode);
        }

        // Adds the poses to low, mid, and high nodes respectively
        if (j == 0 || j == 3) {
          lowNodes.add(currentNode);
        } else if (j == 1 || j == 4) {
          midNodes.add(currentNode);
        } else if (j == 2 || j == 5) {
          highNodes.add(currentNode);
        }

        // Adds the poses to blue nodes or red nodes respectively
        if (j < 3) {
          blueNodes.add(currentNode);
        } else {
          redNodes.add(currentNode);
        }

        // Adds the poses to cone nodes or cube nodes respectively based off row
        if (cone) {
          coneNodes.add(currentNode);
        } else {
          cubeNodes.add(currentNode);
        }
      }

      cone = !cone;
    }

    m_swerveDrive = swerveDrive;
    m_vision = vision;
    m_elevator = elevator;
    m_wrist = wrist;
    m_controls = controls;
  }

  public void initSim() {}

  public Field2d getField2d() {
    return m_field2d;
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
                    m_elevator.getHorizontalTranslation().plus(m_wrist.getHorizontalTranslation()),
                    m_swerveDrive.getHeadingRotation2d()));
    m_field2d.getObject("Intake Pose").setPose(intakePose);
    System.out.println("Wrist Extension: " + Units.metersToInches(m_wrist.getHorizontalTranslation().getX()));
    System.out.println("Horizontal Extension: " + Units.metersToInches(m_swerveDrive.getPoseMeters().getX() - intakePose.getX()));

    m_field2d.getObject("Grid Node").setPoses(gridNodes);

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
  public Pose2d getTargetNode(
      StateHandler.INTAKING_STATES intakeState, Constants.SCORING_STATE mainState) {
    ArrayList<Pose2d> possibleNodes = gridNodes;

    if (m_controls.getAllianceColor() == Alliance.Red) {
      possibleNodes.retainAll(redNodes);
    } else if (m_controls.getAllianceColor() == Alliance.Blue) {
      possibleNodes.retainAll(blueNodes);
    }

    if (intakeState == StateHandler.INTAKING_STATES.CONE) {
      possibleNodes.retainAll(coneNodes);
    } else if (intakeState == StateHandler.INTAKING_STATES.CUBE) {
      possibleNodes.retainAll(cubeNodes);
    }

    if (mainState == Constants.SCORING_STATE.SMART_LOW
        || mainState == Constants.SCORING_STATE.SMART_LOW_REVERSE) {
      possibleNodes.retainAll(lowNodes);
    } else if (mainState == Constants.SCORING_STATE.SMART_MEDIUM) {
      possibleNodes.retainAll(midNodes);
    } else if (mainState == Constants.SCORING_STATE.SMART_HIGH) {
      possibleNodes.retainAll(highNodes);
    }

    // Only works on WPILIB version 2023.3.2 and above
    if (possibleNodes.isEmpty()) return new Pose2d(-1, -1, Rotation2d.fromDegrees(0));
    else return robotPose.nearest(possibleNodes);
  }

  @Override
  public void periodic() {
    updateRobotPoses();

    if (RobotBase.isSimulation()) simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {}
}
