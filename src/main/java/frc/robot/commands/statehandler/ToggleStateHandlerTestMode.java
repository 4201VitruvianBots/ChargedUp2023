// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.statehandler;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StateHandler;

public class ToggleStateHandlerTestMode extends CommandBase {
  /** Creates a new ToggleStateHandlerTestMode. */
  private final StateHandler m_stateHandler;

  private final Command m_defaultCommand;

  private boolean m_testMode = false;

  public ToggleStateHandlerTestMode(StateHandler stateHandler) {
    m_stateHandler = stateHandler;

    m_defaultCommand = m_stateHandler.getDefaultCommand();

    addRequirements(m_stateHandler);


  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_testMode = !m_testMode;

    if (m_testMode) {
      m_stateHandler.setDefaultCommand(new RunStateHandlerTestMode(m_stateHandler));
    } else {
      m_stateHandler.setDefaultCommand(m_defaultCommand);
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
