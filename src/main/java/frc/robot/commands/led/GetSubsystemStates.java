// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of 
// the WPILib BSD license file in the root directory of this project. 
 
package frc.robot.commands.led; 
 
import edu.wpi.first.wpilibj2.command.CommandBase; 
import edu.wpi.first.wpilibj.DriverStation; 
import frc.robot.subsystems.*; 
 
/** Sets the LED based on the subsystems' statuses */ 
public class GetSubsystemStates extends CommandBase { 
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) 
  private final LED m_led; 
 
  private final Wrist m_wrist; 
  private final Intake m_intake; 
  private boolean Cone; 
  private boolean Cube; 
  private boolean disabled; 
  private boolean enabled; 
  private boolean elavating; 
  private boolean intaking; 
  private boolean wrist;  
 
  /** Sets the LED based on the subsystems' statuses */ 
  public GetSubsystemStates( 
      LED led, Intake intake, Wrist wrist) { 
    m_led = led; 
    m_intake = intake; 
    m_wrist = wrist; 
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
  intaking = m_intake.getIntakeState(); 
  wrist = m_wrist.getWristState();  
 
  // set in order of priority to be expressed from the least priority to the 
  // highest priority 
  if (disabled) { 
    m_led.expressState(LED.robotState.DISABLED); 
  } else if (elavating) { 
    m_led.expressState(LED.robotState.ELEVATING); 
  } else if (Cone) { 
    m_led.expressState(LED.robotState.CONE); 
  } else if (Cube) { 
    m_led.expressState(LED.robotState.CUBE); 
  } else if (wrist) { 
    m_led.expressState(LED.robotState.WRIST); 
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