// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.subsystems.wrist.Wrist;

public class AutoSetWristSetpoint extends CommandBase {
  private final Wrist m_wrist;

  private final double m_setpoint;

  /** Creates a new AutoSetWristSetpoint. */
  public AutoSetWristSetpoint(Wrist wrist, double setpoint) {
    m_wrist = wrist;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setSetpointPositionRadians(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // 1 degree = 0.017453293 radians
    return (Math.abs(m_wrist.getPositionRadians() - m_setpoint) < 0.017453293);
  }
}
