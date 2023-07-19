// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.statehandler;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.subsystems.StateHandler;

public class RunStatehandlerTestMode extends CommandBase {
  private final StateHandler m_stateHandler;
  private final SendableChooser<SETPOINT> m_StateChooser = new SendableChooser<>();
  private SETPOINT m_currentState;

  public RunStatehandlerTestMode(StateHandler stateHandler) {

    m_stateHandler = stateHandler;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stateHandler.setIsAutoStowedEnabled(false);
    for (SETPOINT state : SETPOINT.values()) {
      m_StateChooser.addOption(state.toString(), state);
    }

    m_StateChooser.setDefaultOption("STOWED", SETPOINT.STOWED);

    SmartDashboard.putData(" State Selector", m_StateChooser);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentState = m_StateChooser.getSelected();
    m_stateHandler.setDesiredSetpoint(m_StateChooser.getSelected());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
