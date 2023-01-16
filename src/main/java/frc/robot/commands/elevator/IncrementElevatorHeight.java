// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Called when the joystick moves up/down, also acts as manual override
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeights;

public class IncrementElevatorHeight extends CommandBase {
  /** Creates a new IncrementElevatorHeight. */
  private Elevator m_elevator;
  private elevatorHeights heightEnum;
  private double joystickY;

  public IncrementElevatorHeight(Elevator elevator, elevatorHeights heightEnum, double joystickY) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    this.heightEnum = heightEnum;
    this.joystickY = joystickY;
    this.m_elevator = elevator;

    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Elevator.setElevatorDesiredHeightState(heightEnum);
    Elevator.setElevatorJoystickY(joystickY);
    
    Elevator.updateElevatorHeight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
