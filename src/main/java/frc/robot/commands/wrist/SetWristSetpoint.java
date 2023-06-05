// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.subsystems.wrist.Wrist;

import java.util.function.DoubleSupplier;

public class SetWristSetpoint extends CommandBase {
  private final Wrist m_wrist;

  private final DoubleSupplier m_input;
  private final double m_setpoint;

  public SetWristSetpoint(Wrist wrist, double setpoint) {
    this(wrist, setpoint, () -> 0);
  }

  /** Creates a new RunWrist. */
  public SetWristSetpoint(Wrist wrist, double setpoint, DoubleSupplier input) {
    m_wrist = wrist;
    m_setpoint = setpoint;
    m_input = input;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setUserSetpoint(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
    m_wrist.setSetpointPositionRadians(m_setpoint);

    double joystickDeadbandOutput = MathUtil.applyDeadband(m_input.getAsDouble(), 0.1);
    m_wrist.setUserInput(-joystickDeadbandOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setUserSetpoint(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
