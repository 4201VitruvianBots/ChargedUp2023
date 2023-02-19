// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Vision.CAMERA_POSITION;
import frc.robot.simulation.SimConstants.Grids;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.utils.ModuleMap;
import java.util.ArrayList;

public class FieldSim extends SubsystemBase {
  private final SwerveDrive m_swerveDrive;
  private final Vision m_vision;

  private final Field2d m_field2d = new Field2d();

  private static Pose2d robotPose;

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

  public FieldSim(SwerveDrive swerveDrive, Vision vision) {

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

        // Adds the poses to the coopertition node if applicable
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
  }

  public void initSim() {}

  public Field2d getField2d() {
    return m_field2d;
  }

  public void setTrajectory(Trajectory trajectory) {
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
        .setPoses(m_vision.getTagPoses2d(CAMERA_POSITION.LEFT_LOCALIZER));
    m_field2d
        .getObject("lLocalizerPoses")
        .setPoses(m_vision.getRobotPoses2d(Constants.Vision.CAMERA_POSITION.LEFT_LOCALIZER));
    m_field2d
        .getObject("lLocalizerPose")
        .setPose(m_vision.getRobotPose2d(Constants.Vision.CAMERA_POSITION.LEFT_LOCALIZER));
    //    m_field2d
    //        .getObject("Limelight Pose")
    //        .setPose(m_vision.getRobotPose2d(CAMERA_POSITION.RIGHT_LOCALIZER));
    m_field2d
        .getObject("rLocalizerTagPoses")
        .setPoses(m_vision.getTagPoses2d(CAMERA_POSITION.RIGHT_LOCALIZER));
    m_field2d
        .getObject("rLocalizerPoses")
        .setPoses(m_vision.getRobotPoses2d(Constants.Vision.CAMERA_POSITION.RIGHT_LOCALIZER));
    m_field2d
        .getObject("rLocalizerPose")
        .setPose(m_vision.getRobotPose2d(Constants.Vision.CAMERA_POSITION.RIGHT_LOCALIZER));

    //    int i = 1;
    //    for (Pose2d node : gridNodes) {
    //      m_field2d.getObject("Grid Node " + Integer.toString(i)).setPose(node);
    //      i++;
    //    }

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
      StateHandler.intakingStates intakeState, StateHandler.mainRobotStates mainState) {
    ArrayList<Pose2d> possibleNodes = gridNodes;

    if (DriverStation.getAlliance() == Alliance.Red) {
      possibleNodes.retainAll(redNodes);
    } else if (DriverStation.getAlliance() == Alliance.Blue) {
      possibleNodes.retainAll(blueNodes);
    }

    if (intakeState == StateHandler.intakingStates.CONE) {
      possibleNodes.retainAll(coneNodes);
    } else if (intakeState == StateHandler.intakingStates.CUBE) {
      possibleNodes.retainAll(cubeNodes);
    }

    if (mainState == StateHandler.mainRobotStates.SCORE_LOW) {
      possibleNodes.retainAll(lowNodes);
    } else if (mainState == StateHandler.mainRobotStates.SCORE_MEDIUM) {
      possibleNodes.retainAll(midNodes);
    } else if (mainState == StateHandler.mainRobotStates.SCORE_HIGH) {
      possibleNodes.retainAll(highNodes);
    }

    return robotPose.nearest(possibleNodes);
  }

  @Override
  public void periodic() {
    updateRobotPoses();

    if (RobotBase.isSimulation()) simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {}
}
