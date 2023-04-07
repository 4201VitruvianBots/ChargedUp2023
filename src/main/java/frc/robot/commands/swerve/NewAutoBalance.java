// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class NewAutoBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  private boolean isBalancing;
  private final Timer m_timer = new Timer();
  SwerveModuleState[] states;

  public NewAutoBalance(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;

    addRequirements(m_swerveDrive);
  }

  @Override
  public void initialize() {
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0.4;
    double zero = m_swerveDrive.getRollOffsetDegrees();
    if (Math.abs((m_swerveDrive.getRollDegrees() - zero)) > 1) {
      speed = 0.13 * Math.signum(m_swerveDrive.getRollDegrees());
      isBalancing = true;
    } else if (isBalancing) {
      speed = 0;
    }
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(speed, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(speed, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(speed, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(speed, Rotation2d.fromDegrees(90)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
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
