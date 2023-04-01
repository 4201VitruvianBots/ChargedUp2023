// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Called when the joystick moves up/down, also acts as manual override
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.Elevator;

public class AutoSetElevatorSetpoint extends CommandBase {
  /** Creates a new IncrementElevatorHeight. */
  private final Elevator m_elevator;

  private final double m_setpoint;

  public AutoSetElevatorSetpoint(Elevator elevator, double setpoint) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_setpoint = setpoint;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setControlState(ELEVATOR.STATE.CLOSED_LOOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setDesiredPositionMeters(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (m_elevator.getElevatorState() == m_elevatorState) {
    //   interrupted = true;
    // }
  }

  // Returns true when the command should end.
  @Override
  // 1 inch = 0.254 meters
  public boolean isFinished() {
    return (Math.abs(m_elevator.getHeightMeters() - m_setpoint) < 0.0254);
  }
}
