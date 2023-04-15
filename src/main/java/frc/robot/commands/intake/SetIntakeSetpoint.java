// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.INTAKE.INTAKE_SPEEDS;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;

public class SetIntakeSetpoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;
  private final StateHandler m_StateHandler; 

  private final INTAKE_SPEEDS m_speed;

  /** Creates a new RunIntake. */
  public SetIntakeSetpoint(Intake intake, INTAKE_SPEEDS speed, StateHandler statehandler) {
    m_intake = intake;
    m_speed = speed;
    m_StateHandler = statehandler;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speedcheck = m_speed.get(); 
    if (speedcheck > 0.0) {
        m_intake.setIntakeStateCone(true);
        m_intake.setIntakeStateCube(false);
      }
      else if (speedcheck < 0.0) {
        m_intake.setIntakeStateCone(false);
        m_intake.setIntakeStateCube(true);
  } else {
    m_intake.setIntakeStateCone(false);
    m_intake.setIntakeStateCube(false);
  }
}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPercentOutput(m_speed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPercentOutput(0);
    m_intake.setIntakeStateCube(false);
    m_intake.setIntakeStateCone(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
