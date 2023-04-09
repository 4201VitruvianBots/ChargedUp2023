// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.STATE_HANDLER.SUPERSTRUCTURE_STATE;
import frc.robot.subsystems.*;

/*scoring = flashing white, intakingCube = blue,
intakingCone = orange, locked on = flashing green,
enable = green, disabled = red,
cubeButton = purple, coneButton = yellow */

/** Sets the LED based on the subsystems' statuses */
public class GetSubsystemStates extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LEDSubsystem m_led;

  private final StateHandler m_stateHandler;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(LEDSubsystem led, StateHandler stateHandler) {
    m_led = led;
    m_stateHandler = stateHandler;

    addRequirements(m_led);
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
    if (DriverStation.isDisabled()) {
      // TODO: Add an isReady state
      m_led.expressState(SUPERSTRUCTURE_STATE.DISABLED);
    } else {
      switch (m_stateHandler.getDesiredState()) {
          // TODO: Add states for substation intaking
        case INTAKE_LOW_CONE:
          m_led.expressState(SUPERSTRUCTURE_STATE.INTAKE_LOW_CONE);
          break;
        case INTAKE_LOW_CUBE:
          m_led.expressState(SUPERSTRUCTURE_STATE.INTAKE_LOW_CUBE);
          break;
        case ALPHA_ZONE:
        case SCORE_LOW_REVERSE:
        case SCORE_LOW:
          m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_LOW);
          break;
        case SCORE_LOW_CONE:
          m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_LOW_CONE);
          break;
        case SCORE_LOW_CUBE:
          m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_LOW_CUBE);
          break;
        case BETA_ZONE:
        case SCORE_MID_CONE:
        case SCORE_MID_CUBE:
        case SCORE_MID:
          m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_MID);
          break;
        case GAMMA_ZONE:
        case INTAKE_EXTENDED:
        case SCORE_HIGH:
        case SCORE_HIGH_CONE:
        case SCORE_HIGH_CUBE:
          m_led.expressState(SUPERSTRUCTURE_STATE.SCORE_HIGH);
          break;
        default:
          m_led.expressState(SUPERSTRUCTURE_STATE.ENABLED);
          break;
      }
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
