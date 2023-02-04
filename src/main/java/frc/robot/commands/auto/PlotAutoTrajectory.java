// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;

public class PlotAutoTrajectory extends CommandBase {
  FieldSim m_fieldSim;
  Trajectory m_trajectory;
  /** Creates a new PlotAutoTrajectory. */
  public PlotAutoTrajectory(FieldSim fieldSim, Trajectory trajectory) {
    m_fieldSim = fieldSim;
    m_trajectory = trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_fieldSim.setTrajectory(m_trajectory);
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
