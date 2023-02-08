// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateHandler extends SubsystemBase {
  /** Creates a new StateHandler. */
  public enum mainRobotStates {
    STOWED,
    INTAKING,
    AUTO_BALANCE,
    ELEVATING,
    SCORING
  }

  public enum scoringStates {
    NONE,
    UNKNOWN,
    CONE,
    CUBE
  }

  public mainRobotStates currentMainState = mainRobotStates.STOWED;
  public scoringStates currentScoringState = scoringStates.NONE;
  public Elevator.elevatorHeights currentElevatorState = Elevator.elevatorHeights.NONE;

  public StateHandler() {}

  // First part of our state machine
  public void nextState() {}

  // Second part of our state machine
  public void actOnState() {}

  // Final part of our state machine
  public void advanceState() {
    if (Intake.getIntakeState()) {
      currentMainState = mainRobotStates.INTAKING;
    }
    currentElevatorState = Elevator.getElevatorDesiredHeightState();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    nextState();
    actOnState();
    advanceState();
  }
}
