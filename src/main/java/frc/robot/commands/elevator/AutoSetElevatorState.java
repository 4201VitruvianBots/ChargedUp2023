// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Called when the joystick moves up/down, also acts as manual override
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.Elevator.ELEVATOR_STATE;
import frc.robot.subsystems.Elevator;

public class AutoSetElevatorState extends CommandBase {
  /** Creates a new IncrementElevatorHeight. */
  private final Elevator m_elevator;

  private final ELEVATOR_STATE m_elevatorState;

  public AutoSetElevatorState(Elevator elevator, ELEVATOR_STATE state) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_elevatorState = state;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setElevatorState(m_elevatorState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_elevator.getSetpointReached()) {
      interrupted = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
