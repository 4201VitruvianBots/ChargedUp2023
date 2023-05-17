// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class RunElevatorJoystick extends CommandBase {
  /** Creates a new RunElevatorJoystick. This is our default command */
  private final Elevator m_elevator;

  private final DoubleSupplier m_joystickY;

  public RunElevatorJoystick(Elevator elevator, DoubleSupplier joystickY) {
    m_elevator = elevator;
    m_joystickY = joystickY;

    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Adds a Deadband so joystick Ys below 0.05 won't be registered
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.1);

    if (joystickYDeadbandOutput != 0.0) {
      m_elevator.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
      m_elevator.setJoystickY(-joystickYDeadbandOutput);
    }
    if (joystickYDeadbandOutput == 0
        && m_elevator.getClosedLoopControlMode() == CONTROL_MODE.OPEN_LOOP) {
      m_elevator.setDesiredPositionMeters(m_elevator.getHeightMeters());
      m_elevator.resetTrapezoidState();
    }
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
