// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.robotState;

/** Sets the LED based on the subsystems' statuses */
public class IntakeCONE extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LED m_led;

  private final Intake m_intake;
  private boolean Cone;

  private boolean disabled;
  private boolean enabled;

  /** Sets the LED based on the subsystems' statuses */
  public IntakeCONE(LED led, Intake intake, robotState robotState) {
    m_led = led;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.expressState(LED.robotState.ENABLED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // the prioritized state to be expressed to the LEDs
  disabled = DriverStation.isDisabled();
  enabled = !disabled;
  Cone = m_intake.getIntakeState();

  // set in order of priority to be expressed from the least priority to the
  // highest priority
  if (disabled) {
    m_led.expressState(LED.robotState.DISABLED);
  } else if (Cone) {
    m_led.expressState(LED.robotState.CONE);
  } else if (enabled) {
    m_led.expressState(LED.robotState.ENABLED);
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
