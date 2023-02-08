// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristRotations;

public class RunWristJoystick extends CommandBase {
  private Wrist m_Wrist;
  private DoubleSupplier m_JoystickX;
  /** Creates a new RunWristJoystick. */
  public RunWristJoystick(Wrist wrist, DoubleSupplier joystickX) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Wrist = wrist;
    m_JoystickX = joystickX;
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_JoystickX.getAsDouble() != 0.0) {
      m_Wrist.setWristDesiredRotationState(WristRotations.JOYSTICK);
    } else if (m_Wrist.getWristDesiredRotations() == WristRotations.JOYSTICK) {
      m_Wrist.setWristDesiredRotationState(WristRotations.NONE);

    }

    Wrist.setWristJoystickX(m_JoystickX);
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
