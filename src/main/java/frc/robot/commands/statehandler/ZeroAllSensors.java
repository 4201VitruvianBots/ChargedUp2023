// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.statehandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

public class ZeroAllSensors extends CommandBase {
  /** Creates a new ZeroAll. */
  private final SwerveDrive m_swerve;

  private final Elevator m_elevator;
  private final Wrist m_wrist;

  public ZeroAllSensors(SwerveDrive swerveDrive, Elevator elevator, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerveDrive;
    m_elevator = elevator;
    m_wrist = wrist;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetGyro();
    m_elevator.setSensorPosition(0.0);
    m_wrist.resetAngleDegrees(-15.0);
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
