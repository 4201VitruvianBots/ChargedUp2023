// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Called when the joystick moves up/down, also acts as manual override
package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CAN_UTIL_LIMIT;
import frc.robot.subsystems.StateHandler;

public class ToggleCanUtilization extends CommandBase {
  /** Creates a new IncrementElevatorHeight. */
  private StateHandler m_stateHandler;

  public ToggleCanUtilization(StateHandler stateHandler) {
    m_stateHandler = stateHandler;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_stateHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CAN_UTIL_LIMIT limitCan;
    if (m_stateHandler.getReduceCanUtilization() == CAN_UTIL_LIMIT.NORMAL) {
      limitCan = CAN_UTIL_LIMIT.LIMITED;
    } else {
      limitCan = CAN_UTIL_LIMIT.NORMAL;
    }
    m_stateHandler.setReduceCanUtilization(limitCan);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}