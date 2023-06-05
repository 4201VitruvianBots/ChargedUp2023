// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;

public class ResetWristAngleDegrees extends CommandBase {
  private final Wrist m_wrist;

  private final double m_angle;

  /** Creates a new ResetWristAngleDegrees. */
  public ResetWristAngleDegrees(Wrist wrist, double angle) {
    m_wrist = wrist;
    m_angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.resetAngleDegrees(m_angle);
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
