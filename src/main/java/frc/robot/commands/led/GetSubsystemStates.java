// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LED.LED_STATE;
import frc.robot.Constants.STATEHANDLER.INTAKING_STATES;
import frc.robot.Constants.STATEHANDLER.SUPERSTRUCTURE_STATE;
import frc.robot.subsystems.*;

/*scoring = flashing white, intakingcube = blue,
intakingcone = orange, locked on = flashing green,
enable = green, disabled = red,
cubebutton = purple, conebutton = yellow */

/** Sets the LED based on the subsystems' statuses */
public class GetSubsystemStates extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem m_led;
  private final StateHandler m_stateHandler;
  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(
      LEDSubsystem led, StateHandler stateHandler) {
    m_led = led;
    m_stateHandler = stateHandler;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // the prioritized state to be expressed to the LEDs
    // set in order of priority to be expressed from the least priority to the
    // highest priority
      if (m_stateHandler.getCurrentZone() == SUPERSTRUCTURE_STATE.SCORE_HIGH) {
        m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_HIGH);
    } else if (m_stateHandler.getCurrentZone() == SUPERSTRUCTURE_STATE.SCORE_MID) {
        m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_MID);
    } else if (m_stateHandler.getCurrentZone() == SUPERSTRUCTURE_STATE.SCORE_LOW) {
        m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_LOW);
    } else if (m_stateHandler.getCurrentZone() == SUPERSTRUCTURE_STATE.INTAKE_LOW) {
        m_led.expressState(SUPERSTRUCTURE_STATE.INTAKE_LOW);
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
