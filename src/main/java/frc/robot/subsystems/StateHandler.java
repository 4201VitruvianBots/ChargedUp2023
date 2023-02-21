// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simulation.FieldSim;

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
    SCORE_MEDIUM,
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
  public Pose2d targetNode;
  public boolean isOnTarget;

  private final Intake m_Intake;
  private final Wrist m_Wrist;
  private final SwerveDrive m_Drive;
  private final FieldSim m_FieldSim;
  private final Elevator m_Elevator;
  private final LED m_Led;
  private final Vision m_Vision;

  public StateHandler(
      Intake intake,
      Wrist wrist,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Elevator elevator,
      LED led,
      Vision vision) {
    m_Intake = intake;
    m_Drive = swerveDrive;
    m_FieldSim = fieldSim;
    m_Elevator = elevator;
    m_Led = led;
    m_Vision = vision;
    m_Wrist = wrist;
  }

  // First part of our state machine
  public void nextState() {}

  // Second part of our state machine
  public void actOnState() {}

  // Final part of our state machine
  public void advanceState() {
    if (m_Intake.getIntakeState()) {}
    currentElevatorState = Elevator.getElevatorDesiredHeightState();
    // Coordinate Wrist Movement with Elevator Heights
    /*
        switch (currentElevatorState) {
          case JOYSTICK:
            m_wrist.setSetpoint(0.1);
            break;
          case STOWED:
            m_wrist.setSetpoint(0.1);
            break;
          case LOW:
            m_wrist.setSetpoint(0.1);
            break;
          case MID:
            m_wrist.setSetpoint(0.1);
            break;
          case HIGH:
            m_wrist.setSetpoint(0.1);
            break;
        }
    */
  }

  @Override
  public void periodic() {
    targetNode = m_FieldSim.getTargetNode(currentScoringState, currentMainState);
    isOnTarget = m_FieldSim.isRobotOnTarget(targetNode, 0.1);
    switch (currentMainState) {
      case SCORE_HIGH:
        break;

      case SCORE_MEDIUM:
        break;

      case SCORE_LOW:
        break;

      case INTAKING_GROUND:
        break;

      case INTAKING_STATION:
        break;

      case AUTO:
        break;

      case AUTO_BALANCE:
        break;

      case DISABLED:
        break;

      default:
      case STOWED:
        // m_Elevator.setElevatorDesiredHeightState(elevatorHeights.STOWED);
        break;
    }
  }
}
