// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.statehandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.elevator.Elevator;

public class SetSetpoint extends CommandBase {
  /** Creates a new SetSetpoint. */
  private final Elevator m_elevator;

  private final Wrist m_wrist;
  private final StateHandler m_stateHandler;

  private final STATE_HANDLER.SETPOINT m_desiredState;

  public SetSetpoint(
      StateHandler stateHandler,
      Elevator elevator,
      Wrist Wrist,
      STATE_HANDLER.SETPOINT desiredState) {
    m_elevator = elevator;
    m_wrist = Wrist;
    m_stateHandler = stateHandler;
    m_desiredState = desiredState;

    addRequirements(m_elevator, m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
    m_elevator.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
    m_wrist.setUserSetpoint(true);
    m_elevator.setUserSetpoint(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_stateHandler.setDesiredSetpoint(m_desiredState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setUserSetpoint(false);
    m_elevator.setUserSetpoint(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
