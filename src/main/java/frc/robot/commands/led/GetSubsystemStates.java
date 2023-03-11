// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/*scoring = flashing white, intakingcube = blue,
intakingcone = orange, locked on = flashing green,
enable = green, disabled = red,
cubebutton = purple, conebutton = yellow */

/** Sets the LED based on the subsystems' statuses */
public class GetSubsystemStates extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LED m_led;

  private final Intake m_intake;
  private final Controls m_controls;
  private final StateHandler m_stateHandler;
  private boolean intaking;
  private boolean disabled;
  private boolean enabled;
  private boolean elavating;
  private boolean scoring;
  private boolean cubeButton;
  private boolean coneButton;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(LED led, Controls controls, StateHandler stateHandler, Intake intake) {
    m_led = led;
    m_stateHandler = stateHandler;
    m_intake = intake;
    m_controls = controls;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.expressState(LED.robotState.DISABLED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // the prioritized state to be expressed to the LEDs
    disabled = DriverStation.isDisabled();
    enabled = !disabled;
    var desiredState = m_stateHandler.getDesiredZone();

    // set in order of priority to be expressed from the least priority to the
    // highest priority
    if (disabled) {
      if (m_controls.getInitState()) {
        m_led.expressState(LED.robotState.INITIALIZED);
      } else {
        m_led.expressState(LED.robotState.DISABLED);
      }
    } else {
      switch (desiredState) {
        case INTAKE_LOW:
        case INTAKE_EXTENDED:
          m_led.expressState(LED.robotState.INTAKING);
          break;
        case SCORE_LOW_CONE:
        case SCORE_LOW_CUBE:
        case SCORE_MID_CONE:
        case SCORE_MID_CUBE:
        case SCORE_HIGH_CONE:
        case SCORE_HIGH_CUBE:
          if (m_stateHandler.isOnTarget()) {
            m_led.expressState(LED.robotState.LOCKED_ON);
          } else {
            m_led.expressState(LED.robotState.ELEVATING);
          }
          break;
          // TODO: Tie this to Intake Vision Code
          //        case CUBE_BUTTON:
          //          m_led.expressState(LED.robotState.CUBE_BUTTON);
          //          break;
          //        case CONE_BUTTON:
          //          m_led.expressState(LED.robotState.CONE_BUTTON);
          //          break;
        default:
          m_led.expressState(LED.robotState.ENABLED);
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
