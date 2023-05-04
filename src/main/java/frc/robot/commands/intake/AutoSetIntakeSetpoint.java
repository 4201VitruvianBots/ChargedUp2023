// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

public class AutoSetIntakeSetpoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerve;

  private final Intake m_intake;
  private final Vision m_vision;

  private INTAKE.INTAKE_STATE m_state;

  /** Creates a new RunIntake. */
  public AutoSetIntakeSetpoint(
      Intake intake, INTAKE_STATE state, Vision vision, SwerveDrive swerve) {
    m_intake = intake;
    m_state = state;
    m_vision = vision;
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakingState(m_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vision.setPipeline(CAMERA_SERVER.INTAKE, 1.0);
    m_intake.setPercentOutput(m_state.get());
    m_swerve.enableHeadingTarget(true);
    m_swerve.setRobotHeadingRadians(
        m_swerve
            .getHeadingRotation2d()
            .minus(Rotation2d.fromDegrees(m_vision.getTargetXAngle(CAMERA_SERVER.INTAKE)))
            .getRadians());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_state == INTAKE.INTAKE_STATE.INTAKING_CONE)
      m_intake.setIntakingState(INTAKE_STATE.HOLDING_CONE);
    else if (m_state == INTAKE_STATE.INTAKING_CUBE)
      m_intake.setIntakingState(INTAKE_STATE.HOLDING_CUBE);

    m_vision.setPipeline(CAMERA_SERVER.INTAKE, 0);
    m_swerve.enableHeadingTarget(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
