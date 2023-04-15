// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.INTAKE.INTAKE_SPEEDS;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;

public class SetIntakeSetpoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;
  private final StateHandler m_StateHandler; 

  private final INTAKE_SPEEDS m_speed;

  private final Timer m_timer = new Timer();

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
    m_intake.setIntakeStateCube(true);
    m_intake.setIntakeStateCube(true);
    System.out.println("Command started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPercentOutput(m_speed.get());

    //m_StateHandler.StowWrist();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command ended");
    m_intake.setPercentOutput(0);
    m_intake.setIntakeStateCube(false);
    m_intake.setIntakeStateCube(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > 0.2 && m_intake.getFinishedIntaking();
  }
}
