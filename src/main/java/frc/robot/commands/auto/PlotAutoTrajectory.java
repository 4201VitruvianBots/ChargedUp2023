// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;
import java.util.ArrayList;
import java.util.List;

public class PlotAutoTrajectory extends CommandBase {
  private final FieldSim m_fieldSim;
  private final List<PathPlannerTrajectory> m_trajectories;
  private final String m_pathName;

  private final boolean m_useAllianceFlip;

  public PlotAutoTrajectory(
      FieldSim fieldSim, String pathName, List<PathPlannerTrajectory> trajectories) {
    this(fieldSim, pathName, trajectories, true);
  }

  public PlotAutoTrajectory(
      FieldSim fieldSim,
      String pathName,
      List<PathPlannerTrajectory> trajectories,
      boolean useAllainceFlip) {
    m_fieldSim = fieldSim;
    m_pathName = pathName;
    m_trajectories = trajectories;

    m_useAllianceFlip = useAllainceFlip;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_fieldSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<PathPlannerTrajectory> ppTrajectories = new ArrayList<>();
    //    var isRedPath = m_pathName.startsWith("Red");
    //    if (isRedPath && m_useAllianceFlip)
    //      ppTrajectories.addAll(
    //              ppTrajectories.stream().map(t
    // ->SimConstants.allianceFlip((Trajectory)t)).collect(Collectors.toList());
    //    else ppTrajectories.addAll(m_trajectories);
    ppTrajectories.addAll(m_trajectories);

    m_fieldSim.setTrajectory(ppTrajectories);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
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
