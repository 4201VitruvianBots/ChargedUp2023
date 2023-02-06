// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator.elevatorHeights;

public class StateHandler extends SubsystemBase {
  /** Creates a new StateHandler. */

  public enum mainRobotStates {
    DISABLED,
    AUTO,
    STOWED,
    INTAKING_GROUND,
    INTAKING_STATION,
    AUTO_BALANCE,
   SCORE_LOW,
   SOCRE_MEDIUM,
   SCORE_HIGH,
  
  }

  public enum intakingStates {
    NONE,
    INTAKING,
    CONE,
    CUBE
  }

  public mainRobotStates currentMainState = mainRobotStates.STOWED;
  public intakingStates currentScoringState = intakingStates.NONE;
  public Elevator.elevatorHeights currentElevatorState = Elevator.elevatorHeights.STOWED;
private final Intake m_Intake;
private final Wrist m_Wrist;
private final SwerveDrive m_Drive; 
private final FieldSim m_FieldSim;
private final Elevator m_Elevator;
private final LED m_Led;
private final Vision m_Vision;

public StateHandler(
    Intake intake, Wrist wrist, SwerveDrive swerveDrive, FieldSim fieldSim ,Elevator elevator, LED led, Vision vision
  ) {
    m_Intake = intake;
    m_Drive = swerveDrive;
    m_FieldSim = fieldSim;
    m_Elevator = elevator;
    m_Led = led;
    m_Vision = vision;
    m_Wrist = wrist;
  }

  // First part of our state machine
  public void nextState() {

  }

  // Second part of our state machine
  public void actOnState() {

  }

  // Final part of our state machine
  public void advanceState() {
    if (Intake.getIntakeState()) {
    }
    currentElevatorState = Elevator.getElevatorDesiredHeightState();
  }

  @Override
  public void periodic() {
   switch(currentMainState){
  
    case mainRobotStates.SCORE_HIGH:
    break;

    case mainRobotStates.SOCRE_MEDIUM:
    break;
    
    case mainRobotStates.SCORE_LOW:
    break;
    
    case mainRobotStates.INTAKING_GROUND:
    break;
    
    case mainRobotStates.INTAKING_STATION:
    break;
   
    case mainRobotStates.AUTO:
    break;
   
    case mainRobotStates.AUTO_BALANCE:
    break;

    case mainRobotStates.DISABLED:
    break;
    
    default:
    case mainRobotStates.STOWED:
      m_Elevator.setElevatorDesiredHeightState(elevatorHeights.STOWED);
    break;

   }
  }
}
