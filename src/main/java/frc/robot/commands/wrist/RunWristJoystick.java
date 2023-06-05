// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.subsystems.wrist.Wrist;

import java.util.function.DoubleSupplier;

public class RunWristJoystick extends CommandBase {
  private final Wrist m_wrist;

  private final DoubleSupplier m_joystickY;

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
    // Adds a Deadband so joystick Xs below 0.05 won't be registered
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.1);

    if (joystickYDeadbandOutput != 0.0) {
      m_wrist.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
      m_wrist.setUserInput(-joystickYDeadbandOutput);
    }
    if (joystickYDeadbandOutput == 0
        && m_wrist.getClosedLoopControlMode() == CONTROL_MODE.OPEN_LOOP) {
      m_wrist.setSetpointPositionRadians(m_wrist.getPositionRadians());
      m_wrist.resetTrapezoidState();
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
