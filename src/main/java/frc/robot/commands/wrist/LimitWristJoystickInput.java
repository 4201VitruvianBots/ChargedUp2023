// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;

// Limit the percent output of the wrist joystick when the stick is pressed down to make small
// adjustments
public class LimitWristJoystickInput extends CommandBase {
  private final Wrist m_wrist;

  /** Creates a new LimitWristJoystickInput. */
  public LimitWristJoystickInput(Wrist wrist) {
    m_wrist = wrist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setJoystickLimit(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setJoystickLimit(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
