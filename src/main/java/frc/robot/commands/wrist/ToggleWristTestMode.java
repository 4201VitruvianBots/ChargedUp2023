// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.subsystems.Wrist;

public class ToggleWristTestMode extends CommandBase {
  /** Creates a new SetElevatorControlLoop. */
  private final Wrist m_wrist;

  private CONTROL_MODE m_lastcontrolmode;
  private Command m_defultCommand;

  public ToggleWristTestMode(Wrist wrist) {
    m_wrist = wrist;

    addRequirements(m_wrist);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_wrist.getClosedLoopControlMode() != CONTROL_MODE.CLOSED_LOOP_TEST) {
      m_lastcontrolmode = m_wrist.getClosedLoopControlMode();
      m_wrist.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP_TEST);
      m_defultCommand = m_wrist.getDefaultCommand();
      m_wrist.setDefaultCommand(new RunWristTestMode(m_wrist));
    } else {
      m_wrist.setClosedLoopControlMode(m_lastcontrolmode);
      m_wrist.setDefaultCommand(m_defultCommand);
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
