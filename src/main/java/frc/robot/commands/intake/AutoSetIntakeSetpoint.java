// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class AutoSetIntakeSetpoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;

  private final Vision m_vision;
  private final SwerveDrive m_swerve;

  private INTAKE.INTAKE_SPEEDS m_setpoint;

  /** Creates a new RunIntake. */
  public AutoSetIntakeSetpoint(
      Intake intake, INTAKE.INTAKE_SPEEDS setpoint, Vision vision, SwerveDrive swerve) {
    m_intake = intake;
    m_setpoint = setpoint;
    m_vision = vision;
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_setpoint == INTAKE.INTAKE_SPEEDS.INTAKING_CONE) {
      m_intake.setIntakeStateCone(true);
    } else if (m_setpoint == INTAKE.INTAKE_SPEEDS.INTAKING_CUBE) {
      m_intake.setIntakeStateCube(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPercentOutput(m_setpoint.get());
    m_swerve.enableHeadingTarget(true);
    m_swerve.setRobotHeadingRadians(
        m_swerve
            .getHeadingRotation2d()
            .minus(Rotation2d.fromDegrees(m_vision.getTargetXAngle(CAMERA_SERVER.INTAKE)))
            .getRadians());
    System.out.println("We see cube");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_setpoint == INTAKE.INTAKE_SPEEDS.HOLDING_CONE
        || m_setpoint == INTAKE.INTAKE_SPEEDS.INTAKING_CONE) {
      m_intake.setIntakeStateCone(false);
      m_vision.setPipeline(CAMERA_SERVER.INTAKE, 0);
      m_swerve.enableHeadingTarget(false);
    } else if (m_setpoint == INTAKE.INTAKE_SPEEDS.HOLDING_CUBE
        || m_setpoint == INTAKE.INTAKE_SPEEDS.INTAKING_CUBE) {
      m_intake.setIntakeStateCube(false);
      m_vision.setPipeline(CAMERA_SERVER.INTAKE, 0);
      m_swerve.enableHeadingTarget(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
