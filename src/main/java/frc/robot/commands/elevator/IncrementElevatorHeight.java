// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Called when the joystick moves up/down, also acts as manual override
package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeights;

public class IncrementElevatorHeight extends CommandBase {
  /** Creates a new IncrementElevatorHeight. */

  private DoubleSupplier m_joystickY;

  private elevatorHeights heightEnum;

  public IncrementElevatorHeight(elevatorHeights heightEnum, DoubleSupplier joystickY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.heightEnum = heightEnum;
    m_joystickY = joystickY;

    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Elevator.setElevatorDesiredHeightState(heightEnum);
    Elevator.setElevatorJoystickY(m_joystickY);
    
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
