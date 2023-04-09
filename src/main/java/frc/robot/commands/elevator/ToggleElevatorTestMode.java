// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateHandler;

public class ToggleElevatorTestMode extends CommandBase {
  /** Creates a new SetElevatorControlLoop. */
  private final Elevator m_elevator;

  private final StateHandler m_StateHandler;

  private CONTROL_MODE m_lastcontrolmode;
  private Command m_defaultCommand;

  public ToggleElevatorTestMode(Elevator elevator, StateHandler stateHandler) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_StateHandler = stateHandler;
    addRequirements(m_elevator);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.getClosedLoopControlMode() != CONTROL_MODE.CLOSED_LOOP_TEST) {
      m_lastcontrolmode = m_elevator.getClosedLoopControlMode();
      m_elevator.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP_TEST);
      m_defaultCommand = m_elevator.getDefaultCommand();
      m_elevator.setDefaultCommand(new RunElevatorTestMode(m_elevator, m_StateHandler));
    } else {
      m_elevator.setClosedLoopControlMode(m_lastcontrolmode);
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
