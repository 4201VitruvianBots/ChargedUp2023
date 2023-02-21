// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Elevator.ELEVATOR_STATE;
import frc.robot.simulation.FieldSim;

public class StateHandler extends SubsystemBase {
  /** Creates a new StateHandler. */
  public enum MAIN_ROBOT_STATES {
    DISABLED,
    AUTO,
    STOWED,
    INTAKING_GROUND,
    INTAKING_STATION,
    AUTO_BALANCE,
    SCORE_SETPOINT_LOW_INTAKE,
    SCORE_SETPOINT_LOW,
    SCORE_SETPOINT_MEDIUM,
    SCORE_SETPOINT_HIGH,
    SCORE_SMART_LOW,
    SCORE_SMART_MEDIUM,
    SCORE_SMART_HIGH,
  }

  public enum INTAKING_STATES {
    NONE,
    INTAKING,
    CONE,
    CUBE
  }

  public MAIN_ROBOT_STATES currentMainState = MAIN_ROBOT_STATES.STOWED;
  public INTAKING_STATES currentIntakeState = INTAKING_STATES.NONE;
  public MAIN_ROBOT_STATES nextMainState = currentMainState;
  public INTAKING_STATES nextIntakeState = currentIntakeState;
  public Pose2d targetNode;
  public boolean isOnTarget;

  private final Intake m_intake;
  private final Wrist m_wrist;
  private final SwerveDrive m_drive;
  private final FieldSim m_fieldSim;
  private final Elevator m_elevator;
  private final LED m_led;
  private final Vision m_vision;

  public StateHandler(
      Intake intake,
      Wrist wrist,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Elevator elevator,
      LED led,
      Vision vision) {
    m_intake = intake;
    m_drive = swerveDrive;
    m_fieldSim = fieldSim;
    m_elevator = elevator;
    m_led = led;
    m_vision = vision;
    m_wrist = wrist;
  }

  // First part of our state machine
  public void queueMainState(MAIN_ROBOT_STATES state) {
    nextMainState = state;
  }

  public void queueIntakingState(INTAKING_STATES state) {
    nextIntakeState = state;
  }

  public void advanceStates() {}

  @Override
  public void periodic() {
    targetNode = m_fieldSim.getTargetNode(currentIntakeState, currentMainState);
    isOnTarget = false;

    advanceStates();

    // Limit the wrist min/max angle based on where the elevator is.
    // TODO: Make this a linear interpolation
    if (m_elevator.getElevatorHeight()
        > Constants.getInstance().Elevator.elevatorHeightWristLowerLimit) {
      m_wrist.updateWristLowerAngleLimit(Constants.getInstance().Wrist.wristSoftLowerLimitDegrees);
    } else {
      m_wrist.updateWristLowerAngleLimit(Constants.getInstance().Wrist.wristSoftLowerLimitDegrees);
    }
    if (m_elevator.getElevatorHeight()
        > Constants.getInstance().Elevator.elevatorHeightWristUpperLimit) {
      m_wrist.updateWristLowerAngleLimit(
          Constants.getInstance().Wrist.wristAbsoluteUpperLimitDegrees);
    } else {
      m_wrist.updateWristLowerAngleLimit(Constants.getInstance().Wrist.wristSoftUpperLimitDegrees);
    }

    // TODO: Limit max swerve speed by elevator height

    switch (currentIntakeState) {
      case CONE:
        break;
      case CUBE:
        break;
      case INTAKING:
        break;
      default:
      case NONE:
        break;
    }

    switch (currentMainState) {
      case SCORE_SMART_HIGH:
        isOnTarget = m_fieldSim.isRobotOnTarget(targetNode, 0.1);
        break;
      case SCORE_SMART_MEDIUM:
        isOnTarget = m_fieldSim.isRobotOnTarget(targetNode, 0.1);
        break;
      case SCORE_SMART_LOW:
        isOnTarget = m_fieldSim.isRobotOnTarget(targetNode, 0.1);
        break;
      case SCORE_SETPOINT_HIGH:
        break;
      case SCORE_SETPOINT_MEDIUM:
        break;
      case SCORE_SETPOINT_LOW:
        break;
      case SCORE_SETPOINT_LOW_INTAKE:
        m_elevator.setElevatorState(ELEVATOR_STATE.LOW);
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
        m_elevator.setElevatorState(ELEVATOR_STATE.STOWED);
        //        m_Wrist.setWristState(Wrist.WristRotations.);
        break;
    }
  }
}
