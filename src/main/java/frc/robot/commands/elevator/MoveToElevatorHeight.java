// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Called by the shortcut buttons for low, mid, and high elevator heights

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToElevatorHeight extends CommandBase {
  /** Creates a new MoveToElevatorHeight. */
  public MoveToElevatorHeight() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Increasing elevator height by a increment defined
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
    return false;
  }
}
