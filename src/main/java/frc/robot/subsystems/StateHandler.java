// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SCORING_STATE;
import frc.robot.simulation.FieldSim;
import frc.robot.utils.SetpointSolver;

public class StateHandler extends SubsystemBase {
  /** Creates a new StateHandler. */
  public enum INTAKING_STATES {
    NONE,
    INTAKING,
    CONE,
    CUBE
  }

  public enum SUPERSTRUCTURE_STATE {
    STOWED,
    INTAKING,
    LOW_ZONE,
    HIGH_ZONE,
    EXTENDED_ZONE,
    DANGER_ZONE
  }

  public SCORING_STATE scoringState = SCORING_STATE.STOWED;
  public INTAKING_STATES currentIntakeState = INTAKING_STATES.NONE;
  private double m_wristOffset = 0;
  public SUPERSTRUCTURE_STATE m_currentZone = SUPERSTRUCTURE_STATE.STOWED;
  public SUPERSTRUCTURE_STATE m_desiredZone = m_currentZone;
  public SUPERSTRUCTURE_STATE m_nextZone = m_currentZone;
  public Pose2d targetNode;
  private boolean m_zoneEnforcement;
  private boolean m_smartScoringEnabled;
  private boolean m_isOnTarget;

  private final Intake m_intake;
  private final Wrist m_wrist;
  private final SwerveDrive m_drive;
  private final FieldSim m_fieldSim;
  private final Elevator m_elevator;
  private final LED m_led;
  private final Vision m_vision;
  private final SetpointSolver m_setpointSolver;

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
    m_setpointSolver = SetpointSolver.getInstance();
    initSmartDashboard();
  }

  public SUPERSTRUCTURE_STATE getSuperStructureState() {
    return m_currentZone;
  }

  public void enableSmartScoring(boolean enabled) {
    m_smartScoringEnabled = enabled;
  }

  public boolean isOnTarget() {
    return m_isOnTarget;
  }

  public void zoneAdvancement() {
    switch (m_desiredZone) {
    }
    // Advance a state if you pass the zone threshold and are within the next zone's mechanism
    // limits
    switch (m_currentZone) {
      case EXTENDED_ZONE:
        if (m_elevator.getHeightMeters() < Units.inchesToMeters(12)) {
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        }
        break;
      case HIGH_ZONE:
        // TODO: Determine Limit
        if (m_elevator.getHeightMeters() < Units.inchesToMeters(12)) {
          m_currentZone = SUPERSTRUCTURE_STATE.LOW_ZONE;
        } else if (m_elevator.getHeightMeters() > Units.inchesToMeters(24)) {
          m_currentZone = SUPERSTRUCTURE_STATE.EXTENDED_ZONE;
        }
        break;
      case LOW_ZONE:
        // TODO: Determine Limit
        if (m_elevator.getHeightMeters() > Units.inchesToMeters(24)) {
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        }
        break;
      case STOWED:
        // TODO: Determine Limit
        if (m_elevator.getHeightMeters() > Units.inchesToMeters(24)) {
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        }
        m_elevator.setElevatorState(Constants.Elevator.SETPOINT_STATE.STOWED);
        m_wrist.setDesiredAngle(Constants.Wrist.SETPOINT_STATE.STOWED.getValue());
        break;
      case INTAKING:
        m_wrist.setDesiredAngle(Constants.Wrist.SETPOINT_STATE.INTAKING.getValue());
        // TODO: Determine Limit
        if (m_elevator.getHeightMeters() > Units.inchesToMeters(24)) {
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        }
        break;
    }

    //    switch ()
  }

  public boolean checkNextZoneThresholds(double elevatorHeight, SUPERSTRUCTURE_STATE desiredState) {
    return false;
  }

  // Whopper whopper whopper whopper
  // junior double triple whopper
  // flame grilled taste with perfect toppers
  // i rule this day
  public boolean isRobotOnTarget(Pose2d targetPose, double margin) {
    var elevatorPose =
        m_drive
            .getPoseMeters()
            .transformBy(
                new Transform2d(
                    m_elevator.getElevatorTranslation(), m_drive.getHeadingRotation2d()));

    return targetPose.minus(elevatorPose).getTranslation().getNorm() > margin;
  }

  private void initSmartDashboard() {
    var stateHandlerTab = Shuffleboard.getTab("StateHandler");

    //    stateHandlerTab.addDouble("Angle", this::getWristAngleDegrees);
    //    stateHandlerTab.addDouble("Raw position", this::getWristSensorPosition);
    //    stateHandlerTab.addDouble("Setpoint", this::getSetpointDegrees);
  }

  private void updateSmartDashboard() {
    SmartDashboard.putString("Superstructure State", getSuperStructureState().toString());
    //    SmartDashboard.putString("Superstructure State", getSuperStructureState().toString());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    targetNode = m_fieldSim.getTargetNode(currentIntakeState, scoringState);

    // Superstructure states for elevator/wrist limit control. If the elevator is LOW_ZONE,
    // prioritize
    // the elevator limits.
    // If the elevator is HIGH_ZONE, prioritize the wrist limits
    if (m_zoneEnforcement) {
      zoneAdvancement();
      switch (m_currentZone) {
        case EXTENDED_ZONE:
          // Set mechanism soft limits

          break;
        case HIGH_ZONE:
          m_elevator.setUpperLimit(Units.inchesToMeters(12));
          m_elevator.setLowerLimit(Units.inchesToMeters(0));
          m_wrist.setUpperAngleLimit(80);
          m_wrist.setLowerAngleLimit(0);
          break;
        case LOW_ZONE:
          m_wrist.setUpperAngleLimit(80);
          m_wrist.setLowerAngleLimit(-15);
        case STOWED:
          m_elevator.setUpperLimit(Units.inchesToMeters(12));
          m_elevator.setLowerLimit(Units.inchesToMeters(0));
          m_wrist.setUpperAngleLimit(80);
          m_wrist.setLowerAngleLimit(-15);
          break;
        case INTAKING:
          m_elevator.setUpperLimit(Units.inchesToMeters(12));
          m_elevator.setLowerLimit(Units.inchesToMeters(0));
          m_wrist.setUpperAngleLimit(80);
          m_wrist.setLowerAngleLimit(-15);
          break;
      }
    }

    // TODO: Limit max swerve speed by elevator height

    // TODO: Update this based on Intake sensors
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

    if (m_smartScoringEnabled) {
      switch (scoringState) {
        case SMART_HIGH:
          m_wristOffset = Constants.SetpointSolver.WRIST_HORIZONTAL_HIGH_OFFSET;
          m_isOnTarget = isRobotOnTarget(targetNode, 0.1);
          break;
        case SMART_MEDIUM:
          m_wristOffset = Constants.SetpointSolver.WRIST_HORIZONTAL_MID_OFFSET;
          m_isOnTarget = isRobotOnTarget(targetNode, 0.1);
          break;
        case SMART_LOW:
          m_wristOffset = Constants.SetpointSolver.WRIST_HORIZONTAL_LOW_OFFSET;
          m_isOnTarget = isRobotOnTarget(targetNode, 0.1);
      }

      m_setpointSolver.solveSetpoints(
          m_drive.getPoseMeters(),
          m_fieldSim.getTargetNode(currentIntakeState, scoringState),
          scoringState);
      m_wrist.setDesiredAngle(Constants.Wrist.SETPOINT_STATE.HIGH.getValue());
      m_elevator.setElevatorMotionMagicMeters(m_setpointSolver.getElevatorSetpointMeters());
      // TODO: Add this to the SwerveDrive
      // m_drive.setHeadingSetpoint(m_setpointSolver.getChassisSetpointRotation2d());
    }
  }
}
