// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;
import java.util.ArrayList;
import java.util.List;

public class PlotAutoTrajectory extends CommandBase {
  FieldSim m_fieldSim;
  List<PathPlannerTrajectory> m_trajectories;
  PathPlannerTrajectory m_trajectory;
  private String m_pathName;
  boolean useList = false;

  public PlotAutoTrajectory(
      FieldSim fieldSim, String pathName, List<PathPlannerTrajectory> trajectories) {
    m_fieldSim = fieldSim;
    m_pathName = pathName;
    m_trajectories = trajectories;
    useList = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  /** Creates a new PlotAutoTrajectory. */
  public PlotAutoTrajectory(FieldSim fieldSim, String pathName, PathPlannerTrajectory trajectory) {
    m_fieldSim = fieldSim;
    m_pathName = pathName;
    m_trajectory = trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var isRedPath = m_pathName.startsWith("Red");

    if (m_trajectories != null) {
      if (isRedPath) {
        ArrayList<PathPlannerTrajectory> ppTrajectories = new ArrayList<>();
        for (var trajectory : m_trajectories) {
          ppTrajectories.add(
              PathPlannerTrajectory.transformTrajectoryForAlliance(
                  trajectory, DriverStation.Alliance.Red));
        }
        m_fieldSim.setTrajectory(ppTrajectories);
      } else {
        m_fieldSim.setTrajectory(m_trajectories);
      }
    } else {
      if (isRedPath) {
        m_trajectory =
            PathPlannerTrajectory.transformTrajectoryForAlliance(
                m_trajectory, DriverStation.Alliance.Red);
      }
      m_fieldSim.setTrajectory(m_trajectory);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
