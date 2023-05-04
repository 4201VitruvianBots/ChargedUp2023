// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

import java.util.function.DoubleSupplier;

public class IntakeVisionAlignment extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerve;

  private final Vision m_vision;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  public IntakeVisionAlignment(
      Vision vision,
      SwerveDrive swerve,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput) {
    m_vision = vision;
    m_swerve = swerve;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;

    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.searchLimelightTarget(CAMERA_SERVER.INTAKE)) {
      m_swerve.enableHeadingTarget(true);
      m_swerve.setRobotHeadingRadians(
          m_swerve
              .getHeadingRotation2d()
              .plus(Rotation2d.fromDegrees(m_vision.getTargetXAngle(CAMERA_SERVER.INTAKE)))
              .getRadians());
    } else {
      m_swerve.enableHeadingTarget(false);
    }

    double throttle =
        MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()), 0.05)
            * Math.signum(m_throttleInput.getAsDouble());
    double strafe =
        MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()), 0.05)
            * Math.signum(m_strafeInput.getAsDouble());
    double rotation =
        MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()), 0.05)
            * Math.signum(m_rotationInput.getAsDouble());

    m_swerve.drive(throttle, strafe, rotation, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.enableHeadingTarget(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
