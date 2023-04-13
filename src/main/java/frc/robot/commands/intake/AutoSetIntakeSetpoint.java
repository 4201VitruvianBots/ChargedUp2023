// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.INTAKE;
import frc.robot.subsystems.Intake;

public class AutoSetIntakeSetpoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;

  private INTAKE.INTAKE_SPEEDS m_setpoint;

  /** Creates a new RunIntake. */
  public AutoSetIntakeSetpoint(Intake intake, INTAKE.INTAKE_SPEEDS setpoint) {
    m_intake = intake;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_setpoint == INTAKE.INTAKE_SPEEDS.HOLDING_CONE
        || m_setpoint == INTAKE.INTAKE_SPEEDS.INTAKING_CONE) {
      m_intake.setIntakeStateCone(true);
    } else if (m_setpoint == INTAKE.INTAKE_SPEEDS.HOLDING_CUBE
        || m_setpoint == INTAKE.INTAKE_SPEEDS.INTAKING_CUBE) {
      m_intake.setIntakeStateCube(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPercentOutput(m_setpoint.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_setpoint == INTAKE.INTAKE_SPEEDS.HOLDING_CONE
        || m_setpoint == INTAKE.INTAKE_SPEEDS.INTAKING_CONE) {
      m_intake.setIntakeStateCone(false);
    } else if (m_setpoint == INTAKE.INTAKE_SPEEDS.HOLDING_CUBE
        || m_setpoint == INTAKE.INTAKE_SPEEDS.INTAKING_CUBE) {
      m_intake.setIntakeStateCube(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
