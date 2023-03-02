// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.subsystems.*;
import frc.robot.subsystems.StateHandler.SUPERSTRUCTURE_STATE;

/*scoring = flashing white, intakingcube = blue,
intakingcone = orange, locked on = flashing green,
enable = green, disabled = red,
cubebutton = purple, conebutton = yellow */

/** Sets the LED based on the subsystems' statuses */
public class GetSubsystemStates extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LED m_led;

  private final Vision m_vision;
  private final StateHandler m_stateHandler;
  private final Controls m_controls;
  private final Wrist m_wrist;
  private final Intake m_intake;
  private boolean intakingCone;
  private boolean intakingCube;
  private boolean disabled;
  private boolean enabled;
  private boolean elavating;
  private boolean scoring;
  private boolean cubeButton;
  private boolean coneButton;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(
      LED led,
      Controls controls,
      Intake intake,
      Wrist wrist,
      StateHandler stateHandler,
      Vision vision) {
    m_led = led;
    m_controls = controls;
    m_intake = intake;
    m_wrist = wrist;
    m_stateHandler = stateHandler;
    m_vision = vision;
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
    intakingCone = m_vision.getPipeline(CAMERA_SERVER.INTAKE) == 2;
    intakingCube = m_vision.getPipeline(CAMERA_SERVER.INTAKE) == 1;
    scoring = (m_stateHandler.getDesiredZone() == SUPERSTRUCTURE_STATE.EXTENDED_ZONE);
    cubeButton = m_led.getPieceIntent() == LED.PieceType.CUBE;
    coneButton = m_led.getPieceIntent() == LED.PieceType.CONE;

    // set in order of priority to be expressed from the least priority to the
    // highest priority
    if (disabled) {
      if (m_controls.getInitState()) m_led.expressState(LED.robotState.INITIALIZED);
      else m_led.expressState(LED.robotState.DISABLED);
    } else if (cubeButton) {
      m_led.expressState(LED.robotState.CUBE_BUTTON);
    } else if (coneButton) {
      m_led.expressState(LED.robotState.CONE_BUTTON);
    } else if (intakingCone) {
      m_led.expressState(LED.robotState.INTAKINGCONE);
    } else if (intakingCube) {
      m_led.expressState(LED.robotState.INTAKINGCUBE);
    } else if (scoring) {
      m_led.expressState(LED.robotState.SCORING);
    } else if (enabled) {
      m_led.expressState(LED.robotState.ENABLED);
    } else if (elavating) {
      m_led.expressState(LED.robotState.ELEVATING);
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
