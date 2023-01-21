// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.led;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;

/** Sets the LED based on the subsystems' statuses */
public class GetSubsystemStates extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LED m_led;
//TODO: figure out scoring system
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final Vision m_vision;
  private boolean disabled;
  private boolean enabled;
  private boolean intaking;
  private boolean elevating;

  /** Sets the LED based on the subsystems' statuses */
  public GetSubsystemStates(
//TODO: initialize the vision subystem to get status
      LED led, Intake intake, Elevator elevator, Vision vision) {
    m_led = led;
    m_intake = intake;
    m_elevator = elevator;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.expressState(LED.robotState.Enabled);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // the prioritized state to be expressed to the LEDs
    disabled = DriverStation
    enabled = !disabled;
    // intaking = m_intake.getIntakeState(); TODO: wait until method is written
    elevating = m_elevator.getElevatorClimbState();
    //TODO: wait until we have code for identifying cubes and cones in vision

    // set in order of priority to be expressed from the least priority to the
    // highest priority
    if (disabled) {
      m_led.expressState(LED.robotState.Disabled);
    } else if (intaking) {
      m_led.expressState(LED.robotState.Intaking);
    //else if (cone) {
    //m_led.expressState(LED.robotState.Cone);
  //} else if (cube){
    //m_led.expressState(LED.robotState.Cube);
  //}
    } else if (elevating) {
      m_led.expressState(LED.robotState.Elevating);
    }  else if (enabled) {
      m_led.expressState(LED.robotState.Enabled);
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
