// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WRIST;
import frc.robot.subsystems.Wrist;

// TODO: Is this command necessary? If not, remove it
public class SetWristSetpointState extends CommandBase {
  private final Wrist m_wrist;
  private final WRIST.STATE m_state;

  /** Creates a new RunWrist. */
  public SetWristSetpointState(Wrist wrist, WRIST.STATE state) {
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
    m_wrist.setClosedLoopControl(m_state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
