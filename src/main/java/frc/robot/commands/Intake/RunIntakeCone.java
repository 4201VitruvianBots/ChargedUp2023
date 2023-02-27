// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.Vision.CAMERA_LOCATION;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class RunIntakeCone extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;
  private final Vision m_vision;
  private final SwerveDrive m_swerve;

  private double m_PercentOutput;

  /** Creates a new RunIntake. */
  public RunIntakeCone(Intake intake, double PercentOutput, Vision vision, SwerveDrive swerve) {
    m_intake = intake;
    m_vision = vision;
    m_swerve = swerve;
    m_PercentOutput = PercentOutput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeState(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakePercentOutput(-m_PercentOutput);
    if (m_vision.searchLimelightTarget(CAMERA_LOCATION.INTAKE)) {
      m_swerve.enableHeadingTarget(true);
      m_swerve.setRobotHeading(m_vision.getTargetXAngle(CAMERA_LOCATION.INTAKE));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakePercentOutput(0);
    m_intake.setIntakeState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
