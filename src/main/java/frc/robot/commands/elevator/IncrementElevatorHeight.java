// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class IncrementElevatorHeight extends CommandBase {
  /** Creates a new IncrementElevatorHeight. This is our default command */
  private DoubleSupplier m_joystickY;

  private Elevator m_elevator;

  public IncrementElevatorHeight(Elevator elevator, DoubleSupplier joystickY) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_joystickY = joystickY;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setElevatorRunning(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // add '&& Elevator.getElevatorDesiredHeightState() == elevatorHeights.STOWED' to this if
    // statement to prioritize shortcut buttons

    // Deadbands joystick Y so joystick Ys below 0.05 won't be registered
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.1);

    if (joystickYDeadbandOutput != 0.0) {
      m_elevator.setControlState(
          m_elevator.getControlMode()
              ? ELEVATOR.STATE.CLOSED_LOOP_MANUAL
              : ELEVATOR.STATE.OPEN_LOOP_MANUAL);
    } else m_elevator.setControlState(ELEVATOR.STATE.SETPOINT);
    // This else if statement will automatically set the elevator to the STOWED position once the
    // joystick is let go
    // Uncomment if you want to reenable this
    // } else if (m_elevator.getElevatorDesiredHeightState() == elevatorHeights.JOYSTICK) {
    // m_elevator.setElevatorDesiredHeightState(elevatorHeights.STOWED);

    m_elevator.setElevatorJoystickY(-joystickYDeadbandOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setElevatorRunning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
