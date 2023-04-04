// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.Elevator;

public class ToggleElevatorControlMode extends CommandBase {
  /** Creates a new SetElevatorControlLoop. */
  private Elevator m_elevator;

  public ToggleElevatorControlMode(Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;

    addRequirements(m_elevator);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_elevator.getClosedLoopControlState() == ELEVATOR.STATE.CLOSED_LOOP)
      m_elevator.setClosedLoopControlState(ELEVATOR.STATE.OPEN_LOOP_MANUAL);
    else if (m_elevator.getClosedLoopControlState() == ELEVATOR.STATE.OPEN_LOOP_MANUAL)
      m_elevator.setClosedLoopControlState(ELEVATOR.STATE.CLOSED_LOOP);
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
