// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELEVATOR.SETPOINT;
import frc.robot.Constants.LED.LED_STATE;
import frc.robot.subsystems.*;

/** Sets the LED based on the subsystems' statuses */
public class SetScoringState extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem m_led;

  private final SETPOINT m_setpoint;

  /** Sets the LED based on the subsystems' statuses */
  public SetScoringState(LEDSubsystem led, SETPOINT setpoint) {
    m_led = led;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_setpoint == SETPOINT.SCORE_MID_CONE) {
      m_led.expressState(LED_STATE.SCORING);
    } else if (m_setpoint == SETPOINT.SCORE_HIGH_CONE) {
      m_led.expressState(LED_STATE.SCORING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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
