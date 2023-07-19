// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sim.fieldsim;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StateHandler;

public class SwitchTargetNode extends CommandBase {
  /** Creates a new SwitchTargetNode. */
  private final StateHandler m_stateHandler;

  private final boolean m_left;

  public SwitchTargetNode(StateHandler stateHandler, boolean left) {
    m_stateHandler = stateHandler;
    m_left = left;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_stateHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stateHandler.switchTargetNode(m_left);
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
