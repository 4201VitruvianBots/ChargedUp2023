// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.Wrist.WRIST_STATE;
import frc.robot.subsystems.Wrist;

public class SetWristState extends CommandBase {
  private final Wrist m_wrist;
  private WRIST_STATE m_state;

  /** Creates a new RunWrist. */
  public SetWristState(Wrist wrist, WRIST_STATE state) {
    m_wrist = wrist;
    m_state = state;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setWristState(m_state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // If you are not in joystick mode, automatically move back to the stowed position
    if (m_wrist.getWristState() != WRIST_STATE.JOYSTICK) m_wrist.setWristState(WRIST_STATE.STOWED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
