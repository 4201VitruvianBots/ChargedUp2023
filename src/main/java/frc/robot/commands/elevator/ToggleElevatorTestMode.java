// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.elevator.Elevator;

public class ToggleElevatorTestMode extends CommandBase {
  /** Creates a new ToggleElevatorTestMode. */
  private final Elevator m_elevator;

  private final StateHandler m_stateHandler;

  private final Command m_defaultCommand;

  private boolean m_testMode = false;

  public ToggleElevatorTestMode(Elevator elevator, StateHandler stateHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_stateHandler = stateHandler;

    m_defaultCommand = m_elevator.getDefaultCommand();

    addRequirements(m_elevator);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_testMode = !m_testMode;

    m_elevator.setTestMode(m_testMode);

    if (m_testMode) {
      m_elevator.setDefaultCommand(new RunElevatorTestMode(m_elevator, m_stateHandler));
    } else {
      m_elevator.setDefaultCommand(m_defaultCommand);
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
