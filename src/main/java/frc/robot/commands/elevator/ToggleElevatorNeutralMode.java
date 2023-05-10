// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class ToggleElevatorNeutralMode extends CommandBase {
  /** Creates a new ToggleElevatorCoastMode. */
  private final Elevator m_elevator;

  public ToggleElevatorNeutralMode(Elevator elevator) {
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NeutralMode neutralMode = m_elevator.getNeutralMode();
    if (neutralMode == NeutralMode.Coast) {
      m_elevator.setNeutralMode(NeutralMode.Brake);
    } else if (neutralMode == NeutralMode.Brake) {
      m_elevator.setNeutralMode(NeutralMode.Coast);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
