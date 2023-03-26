// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Called when the joystick moves up/down, also acts as manual override
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.Elevator;

// TODO: Is this necessary? If not, remove it
public class SetElevatorDesiredSetpointState extends CommandBase {
  /** Creates a new IncrementElevatorHeight. */
  private final Elevator m_elevator;

  private final ELEVATOR.STATE m_setpoint;

  public SetElevatorDesiredSetpointState(Elevator elevator, ELEVATOR.STATE state) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_setpoint = state;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setIsElevating(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setControlState(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setIsElevating(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
