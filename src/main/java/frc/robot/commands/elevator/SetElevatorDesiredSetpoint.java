// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Called when the joystick moves up/down, also acts as manual override
package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class SetElevatorDesiredSetpoint extends CommandBase {
  /** Creates a new IncrementElevatorHeight. */
  private final Elevator m_elevator;

  private final DoubleSupplier m_input;

  private final double m_setpoint;

  public SetElevatorDesiredSetpoint(Elevator elevator, double setpoint) {
    this(elevator, setpoint, () -> 0);
  }

  public SetElevatorDesiredSetpoint(Elevator elevator, double setpoint, DoubleSupplier input) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_setpoint = setpoint;
    m_input = input;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setElevatorRunning(true);
    m_elevator.setControlState(ELEVATOR.STATE.SETPOINT);
    m_elevator.setDesiredPositionMeters(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_input.getAsDouble(), 0.1);

    m_elevator.setDesiredPositionMeters(
        m_setpoint + joystickYDeadbandOutput * m_elevator.setpointMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setElevatorRunning(false);
    m_elevator.setDesiredPositionMeters(ELEVATOR.SETPOINT.STOWED.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
