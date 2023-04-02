// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.statehandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.STATEHANDLER;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.Wrist;

public class SetSetpoint extends CommandBase {
  /** Creates a new SetStatehandlerstate. */
  private Elevator m_Elevator;

  private Wrist m_Wrist;
  private StateHandler m_StateHandler;
  private STATEHANDLER.SETPOINT m_desiredState;

  public SetSetpoint(
      StateHandler stateHandler,
      Elevator elevator,
      Wrist Wrist,
      STATEHANDLER.SETPOINT desiredState) {
    m_Elevator = elevator;
    m_Wrist = Wrist;
    m_StateHandler = stateHandler;
    m_desiredState = desiredState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_StateHandler.setDesiredSetpoint(m_desiredState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
