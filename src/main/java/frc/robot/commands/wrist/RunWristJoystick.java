// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WRIST;
import frc.robot.subsystems.Wrist;
import java.util.function.DoubleSupplier;

public class RunWristJoystick extends CommandBase {
  private Wrist m_wrist;
  private DoubleSupplier m_joystickY;
  /** Creates a new RunWristJoystick. */
  public RunWristJoystick(Wrist wrist, DoubleSupplier joystickY) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    m_joystickY = joystickY;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // commands to move wrist (Move using joystick values, go to a setpoint)
  @Override
  public void execute() {
    // Deadbands joystick X so joystick Xs below 0.05 won't be registered
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.1);

    if (Math.abs(joystickYDeadbandOutput) != 0) {
      //      m_wrist.setControlState(
      //          m_wrist.getClosedLoopState()
      //              ? WRIST.STATE.CLOSED_LOOP_MANUAL
      //              : WRIST.STATE.OPEN_LOOP_MANUAL);
      if (m_wrist.getControlState() == WRIST.STATE.USER_SETPOINT) {
        m_wrist.setUserInput(-joystickYDeadbandOutput);
      } else {
        m_wrist.setControlState(WRIST.STATE.OPEN_LOOP_MANUAL);
        m_wrist.setUserInput(-joystickYDeadbandOutput);
      }
    }
    if (joystickYDeadbandOutput == 0 && m_wrist.getControlState() == WRIST.STATE.OPEN_LOOP_MANUAL) {
      m_wrist.setSetpointPositionRadians(m_wrist.getPositionRadians());
      m_wrist.resetState();
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
