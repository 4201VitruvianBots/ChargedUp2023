// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sim.fieldsim;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;

public class ToggleTestIntakeState extends CommandBase {
  /** Creates a new SetElevatorControlLoop. */
  private final FieldSim m_fieldSim;

  public ToggleTestIntakeState(FieldSim fieldSim) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_fieldSim = fieldSim;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_fieldSim.setTestScoringState(!m_fieldSim.getTestScoringState());
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
