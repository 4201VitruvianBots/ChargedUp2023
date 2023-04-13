// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.subsystems.Intake;

public class SetIntakeMode extends CommandBase {
  /** Creates a new SetCubeMode. */
  private final Intake m_intake;

  private final INTAKE_STATE m_mode;

  public SetIntakeMode(Intake intake, INTAKE_STATE mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakeMode(m_mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeMode(INTAKE_STATE.CONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
