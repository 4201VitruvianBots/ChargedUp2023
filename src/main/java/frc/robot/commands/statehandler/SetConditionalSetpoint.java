// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.statehandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.elevator.Elevator;

public class SetConditionalSetpoint extends CommandBase {
  /** Creates a new SetConditionalSetpoint. */
  private final StateHandler m_stateHandler;

  private final Elevator m_elevator;
  private final Wrist m_wrist;
  private final Intake m_intake;

  private final SCORING_STATE m_desiredState;

  public SetConditionalSetpoint(
      StateHandler stateHandler,
      Elevator elevator,
      Wrist Wrist,
      Intake intake,
      SCORING_STATE desiredState) {
    m_stateHandler = stateHandler;
    m_elevator = elevator;
    m_wrist = Wrist;
    m_intake = intake;

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
    switch (m_desiredState) {
      case INTAKE_EXTENDED:
        if (m_intake.isUsingCubeSetpoint())
          m_stateHandler.setDesiredSetpoint(SETPOINT.INTAKING_EXTENDED_CUBE);
        else m_stateHandler.setDesiredSetpoint(SETPOINT.INTAKING_EXTENDED_CONE);
        break;
      case HIGH:
        if (m_intake.isUsingCubeSetpoint())
          m_stateHandler.setDesiredSetpoint(SETPOINT.SCORE_HIGH_CUBE);
        else m_stateHandler.setDesiredSetpoint(SETPOINT.SCORE_HIGH_CONE);
        break;
      case MID:
        if (m_intake.isUsingCubeSetpoint())
          m_stateHandler.setDesiredSetpoint(SETPOINT.SCORE_MID_CUBE);
        else m_stateHandler.setDesiredSetpoint(SETPOINT.SCORE_MID_CONE);
        break;
      case LOW:
        m_stateHandler.setDesiredSetpoint(SETPOINT.SCORE_LOW);
        break;
      case LOW_REVERSE:
        m_stateHandler.setDesiredSetpoint(SETPOINT.SCORE_LOW_REVERSE);
        break;
    }
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
