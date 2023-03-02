// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.Constants.WRIST;
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
    INTAKE_LOW,
    SCORE_LOW_REVERSE,
    LOW_ZONE,
    HIGH_ZONE,
    EXTENDED_ZONE,
    INTAKE_EXTENDED,
    SCORE_LOW,
    SCORE_MID,
    SCORE_HIGH,
    DANGER_ZONE
  }

  public enum ZONE_TRANSITIONS {
    NONE,
    LOW_TO_HIGH,
    HIGH_TO_LOW,
    HIGH_TO_EXTENDED,
    EXTENDED_TO_HIGH
  }

  public SCORING_STATE scoringState = SCORING_STATE.STOWED;
  public INTAKING_STATES currentIntakeState = INTAKING_STATES.NONE;
  private double m_wristOffset = 0;
  public SUPERSTRUCTURE_STATE m_currentZone = SUPERSTRUCTURE_STATE.STOWED;
  public SUPERSTRUCTURE_STATE m_desiredZone = m_currentZone;
  public ZONE_TRANSITIONS m_nextZone = ZONE_TRANSITIONS.NONE;
  public Pose2d targetNode;
  private boolean m_zoneEnforcement = true;
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

  private StringPublisher m_currentStatePub, m_desiredStatePub;
  private DoublePublisher m_elevatorHeightPub,
      m_elevatorLowerLimPub,
      m_elevatorUpperLimPub,
      m_wristAnglePub,
      m_wristLowerLimPub,
      m_wristUpperLimPub;

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

  public SUPERSTRUCTURE_STATE getCurrentZone() {
    return m_currentZone;
  }

  public SUPERSTRUCTURE_STATE getDesiredZone() {
    return m_desiredZone;
  }

  public void enableSmartScoring(boolean enabled) {
    m_smartScoringEnabled = enabled;
  }

  public boolean isOnTarget() {
    return m_isOnTarget;
  }

  public SUPERSTRUCTURE_STATE determineSuperStructureState(
      double elevatorPositionMeters, double wristPositionRadians) {
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.STOWED.get()) < Units.inchesToMeters(2)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.STOWED.get()) < Units.degreesToRadians(5))
      return SUPERSTRUCTURE_STATE.STOWED;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.INTAKING_LOW.get())
            < Units.inchesToMeters(2)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.INTAKING_LOW.get())
            < Units.degreesToRadians(5)) return SUPERSTRUCTURE_STATE.INTAKE_LOW;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_LOW_REVERSE.get())
            < Units.inchesToMeters(2)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_LOW_REVERSE.get())
            < Units.degreesToRadians(5)) return SUPERSTRUCTURE_STATE.SCORE_LOW_REVERSE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.INTAKING_EXTENDED.get())
            < Units.inchesToMeters(2)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.INTAKING_EXTENDED.get())
            < Units.degreesToRadians(5)) return SUPERSTRUCTURE_STATE.INTAKE_EXTENDED;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_LOW.get())
            < Units.inchesToMeters(2)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_LOW.get())
            < Units.degreesToRadians(5)) return SUPERSTRUCTURE_STATE.SCORE_LOW;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_MID.get())
            < Units.inchesToMeters(2)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_MID.get())
            < Units.degreesToRadians(5)) return SUPERSTRUCTURE_STATE.SCORE_MID;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_HIGH.get())
            < Units.inchesToMeters(2)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_HIGH.get())
            < Units.degreesToRadians(5)) return SUPERSTRUCTURE_STATE.SCORE_HIGH;

    if (elevatorPositionMeters <= ELEVATOR.THRESHOLD.LOW_MAX.get()) {
      if (WRIST.THRESHOLD.LOW_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.LOW_MAX.get())
        return SUPERSTRUCTURE_STATE.LOW_ZONE;
    }
    if (ELEVATOR.THRESHOLD.HIGH_MIN.get() < elevatorPositionMeters
        && elevatorPositionMeters <= ELEVATOR.THRESHOLD.HIGH_MAX.get()) {
      if (WRIST.THRESHOLD.HIGH_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.HIGH_MAX.get())
        return SUPERSTRUCTURE_STATE.HIGH_ZONE;
    }
    if (ELEVATOR.THRESHOLD.EXTENDED_MIN.get() < elevatorPositionMeters
        && elevatorPositionMeters <= ELEVATOR.THRESHOLD.EXTENDED_MAX.get()) {
      if (WRIST.THRESHOLD.EXTENDED_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.EXTENDED_MAX.get())
        return SUPERSTRUCTURE_STATE.EXTENDED_ZONE;
    }
    return SUPERSTRUCTURE_STATE.DANGER_ZONE;
  }

  public void zoneAdvancement() {
    // If your current zone is not equal to your desired zone, assume you are transitioning between
    // zones
    if (m_desiredZone != m_currentZone) {
      if (m_currentZone == SUPERSTRUCTURE_STATE.EXTENDED_ZONE) {
        if (m_desiredZone.ordinal() < m_currentZone.ordinal())
          m_nextZone = ZONE_TRANSITIONS.EXTENDED_TO_HIGH;
      } else if (m_currentZone == SUPERSTRUCTURE_STATE.HIGH_ZONE) {
        if (m_desiredZone.ordinal() > m_currentZone.ordinal())
          m_nextZone = ZONE_TRANSITIONS.HIGH_TO_EXTENDED;
        else if (m_desiredZone.ordinal() < m_currentZone.ordinal())
          m_nextZone = ZONE_TRANSITIONS.LOW_TO_HIGH;
      } else if (m_currentZone == SUPERSTRUCTURE_STATE.LOW_ZONE) {
        if (m_desiredZone.ordinal() > m_currentZone.ordinal())
          m_nextZone = ZONE_TRANSITIONS.LOW_TO_HIGH;
      }
    } else m_nextZone = ZONE_TRANSITIONS.NONE;

    // Use zone transition info to set mechanism limits. Only threshold limit when within transition
    // zones
    switch (m_nextZone) {
      case LOW_TO_HIGH:
        if (Math.abs(m_elevator.getHeightMeters() - ELEVATOR.THRESHOLD.LOW_TO_HIGH.get())
            < Units.inchesToMeters(4)) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.LOW_MIN.get(), ELEVATOR.THRESHOLD.HIGH_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.LOW_MAX.get(), ELEVATOR.THRESHOLD.HIGH_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.LOW_MIN.get(), WRIST.THRESHOLD.HIGH_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.LOW_MAX.get(), WRIST.THRESHOLD.HIGH_MAX.get()));
        }
      case HIGH_TO_LOW:
        if (Math.abs(m_elevator.getHeightMeters() - ELEVATOR.THRESHOLD.HIGH_TO_LOW.get())
            < Units.inchesToMeters(4)) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.LOW_MIN.get(), ELEVATOR.THRESHOLD.HIGH_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.LOW_MAX.get(), ELEVATOR.THRESHOLD.HIGH_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.LOW_MIN.get(), WRIST.THRESHOLD.HIGH_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.LOW_MAX.get(), WRIST.THRESHOLD.HIGH_MAX.get()));
        }
      case HIGH_TO_EXTENDED:
        if (Math.abs(m_elevator.getHeightMeters() - ELEVATOR.THRESHOLD.HIGH_TO_EXTENDED.get())
            < Units.inchesToMeters(4)) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.HIGH_MIN.get(), ELEVATOR.THRESHOLD.EXTENDED_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.HIGH_MAX.get(), ELEVATOR.THRESHOLD.EXTENDED_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.HIGH_MIN.get(), WRIST.THRESHOLD.EXTENDED_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.HIGH_MAX.get(), WRIST.THRESHOLD.EXTENDED_MAX.get()));
        }
      case EXTENDED_TO_HIGH:
        if (Math.abs(m_elevator.getHeightMeters() - ELEVATOR.THRESHOLD.EXTENDED_TO_HIGH.get())
            < Units.inchesToMeters(4)) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.HIGH_MIN.get(), ELEVATOR.THRESHOLD.EXTENDED_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.HIGH_MAX.get(), ELEVATOR.THRESHOLD.EXTENDED_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.HIGH_MIN.get(), WRIST.THRESHOLD.EXTENDED_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.HIGH_MAX.get(), WRIST.THRESHOLD.EXTENDED_MAX.get()));
        }
      default:
      case NONE:
        switch (m_currentZone) {
          case LOW_ZONE:
          case INTAKE_LOW:
          case STOWED:
            m_elevator.setLowerLimitMeters(ELEVATOR.THRESHOLD.LOW_MIN.get());
            m_elevator.setUpperLimitMeters(ELEVATOR.THRESHOLD.LOW_MAX.get());
            m_wrist.setLowerLimit(WRIST.THRESHOLD.LOW_MIN.get());
            m_wrist.setUpperLimit(WRIST.THRESHOLD.LOW_MAX.get());
            break;
          case HIGH_ZONE:
            m_elevator.setLowerLimitMeters(ELEVATOR.THRESHOLD.HIGH_MIN.get());
            m_elevator.setUpperLimitMeters(ELEVATOR.THRESHOLD.HIGH_MAX.get());
            m_wrist.setLowerLimit(WRIST.THRESHOLD.HIGH_MIN.get());
            m_wrist.setUpperLimit(WRIST.THRESHOLD.HIGH_MAX.get());
            break;
          case EXTENDED_ZONE:
          case INTAKE_EXTENDED:
            m_elevator.setLowerLimitMeters(ELEVATOR.THRESHOLD.EXTENDED_MIN.get());
            m_elevator.setUpperLimitMeters(ELEVATOR.THRESHOLD.EXTENDED_MAX.get());
            m_wrist.setLowerLimit(WRIST.THRESHOLD.EXTENDED_MIN.get());
            m_wrist.setUpperLimit(WRIST.THRESHOLD.EXTENDED_MAX.get());
            break;
        }
        break;
    }

    // Check current mechanism positions before advancing zones
    switch (m_currentZone) {
      case EXTENDED_ZONE:
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.EXTENDED_TO_HIGH.get())
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        break;
      case HIGH_ZONE:
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.HIGH_TO_LOW.get())
          m_currentZone = SUPERSTRUCTURE_STATE.LOW_ZONE;
        if (m_elevator.getHeightMeters() > ELEVATOR.THRESHOLD.HIGH_TO_EXTENDED.get())
          m_currentZone = SUPERSTRUCTURE_STATE.EXTENDED_ZONE;
        break;
      case LOW_ZONE:
        if (m_elevator.getHeightMeters() > ELEVATOR.THRESHOLD.LOW_TO_HIGH.get())
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        break;
    }
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
                    m_elevator.getElevatorField2dTranslation(), m_drive.getHeadingRotation2d()));

    return targetPose.minus(elevatorPose).getTranslation().getNorm() > margin;
  }

  private void initSmartDashboard() {
    var stateHandlerTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("StateHandler");
    m_currentStatePub = stateHandlerTab.getStringTopic("currentState").publish();
    m_desiredStatePub = stateHandlerTab.getStringTopic("desiredState").publish();
    m_elevatorHeightPub = stateHandlerTab.getDoubleTopic("elevatorHeightInches").publish();
    m_elevatorLowerLimPub = stateHandlerTab.getDoubleTopic("elevatorMinLimit").publish();
    m_elevatorUpperLimPub = stateHandlerTab.getDoubleTopic("elevatorMaxLimit").publish();
    m_wristAnglePub = stateHandlerTab.getDoubleTopic("wristAngleDegrees").publish();
    m_wristLowerLimPub = stateHandlerTab.getDoubleTopic("wristMinLimit").publish();
    m_wristUpperLimPub = stateHandlerTab.getDoubleTopic("wristMaxLimit").publish();
    //    stateHandlerTab.addDouble("Angle", this::getWristAngleDegrees);
    //    stateHandlerTab.addDouble("Raw position", this::getWristSensorPosition);
    //    stateHandlerTab.addDouble("Setpoint", this::getSetpointDegrees);
  }

  private void updateSmartDashboard() {
    SmartDashboard.putString("Superstructure State", getCurrentZone().toString());
    //    SmartDashboard.putString("Superstructure State", getSuperStructureState().toString());
    m_currentStatePub.set(getCurrentZone().toString());
    m_desiredStatePub.set(getDesiredZone().toString());
    m_elevatorHeightPub.set(Units.metersToInches(m_elevator.getHeightMeters()));
    m_elevatorUpperLimPub.set(Units.metersToInches(m_elevator.getUpperLimitMeters()));
    m_elevatorLowerLimPub.set(Units.metersToInches(m_elevator.getLowerLimitMeters()));
    m_wristAnglePub.set(m_wrist.getPositionDegrees());
    m_wristUpperLimPub.set(Units.radiansToDegrees(m_wrist.getUpperLimit()));
    m_wristLowerLimPub.set(Units.radiansToDegrees(m_wrist.getLowerLimit()));
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    targetNode = m_fieldSim.getTargetNode(currentIntakeState, scoringState);

    // Determine current zone based on elevator/wrist position
    m_currentZone =
        determineSuperStructureState(m_elevator.getHeightMeters(), m_wrist.getPositionRadians());
    //    DriverStation.reportError(
    //            "StateHandler - Superstructure is in an undefined state. ElevatorHeightMeters: "
    //                    + m_elevator.getHeightMeters()
    //                    + "\tWristAngleDegrees: "
    //                    + Units.radiansToDegrees(m_wrist.getPositionRadians()),
    //            false);

    // Determine desired zone based on elevator/wrist setpoints
    m_desiredZone =
        determineSuperStructureState(
            m_elevator.getDesiredPositionMeters(), m_wrist.getDesiredPositionRadians());
    //    if (m_desiredZone == SUPERSTRUCTURE_STATE.DANGER_ZONE)
    //      DriverStation.reportWarning(
    //              "StateHandler - Desired State is not defined. ElevatorDesiredPositionMeters: "
    //                      + m_elevator.getDesiredPositionMeters()
    //                      + "\tWristDesiredPositionDegrees: "
    //                      + Units.radiansToDegrees(m_wrist.getDesiredPositionRadians()),
    //              false);

    // Limit wrist/elevator setpoints to safe thresholds based on where you are and where you want
    // to go
    if (m_zoneEnforcement) {
      zoneAdvancement();
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
          m_wristOffset = Constants.STATEHANDLER.WRIST_SETPOINT_OFFSET.HIGH.get();
          m_isOnTarget = isRobotOnTarget(targetNode, 0.1);
          break;
        case SMART_MEDIUM:
          m_wristOffset = Constants.STATEHANDLER.WRIST_SETPOINT_OFFSET.MID.get();
          m_isOnTarget = isRobotOnTarget(targetNode, 0.1);
          break;
        case SMART_LOW:
          m_wristOffset = Constants.STATEHANDLER.WRIST_SETPOINT_OFFSET.LOW.get();
          m_isOnTarget = isRobotOnTarget(targetNode, 0.1);
      }

      m_setpointSolver.solveSetpoints(
          m_drive.getPoseMeters(),
          m_fieldSim.getTargetNode(currentIntakeState, scoringState),
          scoringState);
      m_wrist.setDesiredPositionRadians(WRIST.SETPOINT.SCORE_HIGH.get());
      m_elevator.setElevatorMotionMagicMeters(m_setpointSolver.getElevatorSetpointMeters());
      // TODO: Add this to the SwerveDrive
      // m_drive.setHeadingSetpoint(m_setpointSolver.getChassisSetpointRotation2d());
    }
  }
}
