// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class RunWrist extends CommandBase {
  private final Wrist m_wrist;
  private double m_percentOutput;

  /** Creates a new RunWrist. */
  public RunWrist(Wrist wrist, double PercentOutput) {
    m_wrist = wrist;
    m_percentOutput = PercentOutput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setWristState(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setWristPercentOutput(m_percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setWristState(false);
    m_wrist.setWristPercentOutput(0);
    m_wrist.setWristState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
