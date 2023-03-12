// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.STATEHANDLER.INTAKING_STATES;
import frc.robot.subsystems.*;

/** Sets the LED based on the subsystems' statuses */
public class SetPieceTypeIntent extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem m_led;

  private final INTAKING_STATES m_pieceType;

  /** Sets the LED based on the subsystems' statuses */
  public SetPieceTypeIntent(LEDSubsystem led, INTAKING_STATES piecetype) {
    m_led = led;
    m_pieceType = piecetype;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.setPieceIntent(m_pieceType);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_led.setPieceIntent(INTAKING_STATES.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
