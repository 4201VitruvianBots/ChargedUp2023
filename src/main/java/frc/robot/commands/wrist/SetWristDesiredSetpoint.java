// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WRIST;
import frc.robot.subsystems.Wrist;
import java.util.function.DoubleSupplier;

public class SetWristDesiredSetpoint extends CommandBase {
  private final Wrist m_wrist;
  private double m_setpoint;
  private final DoubleSupplier m_input;

  public SetWristDesiredSetpoint(Wrist wrist, double setpoint) {
    this(wrist, setpoint, () -> 0);
  }

  /** Creates a new RunWrist. */
  public SetWristDesiredSetpoint(Wrist wrist, double setpoint, DoubleSupplier input) {
    m_wrist = wrist;
    m_setpoint = setpoint;
    m_input = input;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_wrist.setControlState(WRIST.STATE.USER_SETPOINT);
    // m_wrist.setDesiredPositionRadians(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setControlState(WRIST.STATE.USER_SETPOINT);
    m_wrist.setDesiredPositionRadians(m_setpoint);

    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_input.getAsDouble(), 0.1);
    m_wrist.setUserInput(-joystickYDeadbandOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setControlState(WRIST.STATE.AUTO_SETPOINT);
    m_wrist.setDesiredPositionRadians(WRIST.SETPOINT.STOWED.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
