// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntakeCube extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;

  private double m_PercentOutput;

  /** Creates a new RunIntake. */
  public RunIntakeCube(Intake intake, double PercentOutput) {
    m_intake = intake;
    m_PercentOutput = PercentOutput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setBooleanState(true);
    m_intake.setIntakeStateCube(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPercentOutput(m_PercentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPercentOutput(0.3);
    m_intake.setBooleanState(false);
    m_intake.setIntakeStateCube(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
