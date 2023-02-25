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
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ELEVATOR;
import frc.robot.constants.Constants.SCORING_STATE;
import frc.robot.constants.Constants.WRIST;
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
    INTAKE_EXTENDED,
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

  private StringPublisher m_statePub;
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

  public SUPERSTRUCTURE_STATE getSuperStructureState() {
    return m_currentZone;
  }

  public void enableSmartScoring(boolean enabled) {
    m_smartScoringEnabled = enabled;
  }

  public boolean isOnTarget() {
    return m_isOnTarget;
  }

  public SUPERSTRUCTURE_STATE determineDesiredSuperStructureState(
      double elevatorDesiredSetpoint, double wristDesiredSetpoint) {
    if (elevatorDesiredSetpoint <= 6) {
      if (-15 < wristDesiredSetpoint && wristDesiredSetpoint <= 30)
        return SUPERSTRUCTURE_STATE.INTAKE_LOW;
      else if (30 < wristDesiredSetpoint && wristDesiredSetpoint <= 60)
        return SUPERSTRUCTURE_STATE.STOWED;
      else if (60 < wristDesiredSetpoint && wristDesiredSetpoint <= 80)
        return SUPERSTRUCTURE_STATE.LOW_ZONE;
    } else if (6 < elevatorDesiredSetpoint && elevatorDesiredSetpoint <= 12) {
      return SUPERSTRUCTURE_STATE.HIGH_ZONE;
    } else if (24 < elevatorDesiredSetpoint) {
      if (30 < elevatorDesiredSetpoint
          && elevatorDesiredSetpoint <= 36
          && 160 < wristDesiredSetpoint
          && wristDesiredSetpoint <= 200) return SUPERSTRUCTURE_STATE.INTAKE_EXTENDED;
      else return SUPERSTRUCTURE_STATE.EXTENDED_ZONE;
    }
    return SUPERSTRUCTURE_STATE.DANGER_ZONE;
  }

  public void zoneAdvancement() {
    m_desiredZone = determineDesiredSuperStructureState(0, m_wrist.getDesiredAngle());
    // Advance a state if you pass the zone threshold and are within the next zone's mechanism
    // limits
    if (m_desiredZone != m_currentZone) {}

    switch (m_currentZone) {
      case EXTENDED_ZONE:
        if (m_elevator.getHeightMeters() < Units.inchesToMeters(20)) {
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        }
        break;
      case HIGH_ZONE:
        if (m_elevator.getHeightMeters() < Units.inchesToMeters(12)) {
          m_currentZone = SUPERSTRUCTURE_STATE.LOW_ZONE;
        } else if (m_elevator.getHeightMeters() > Units.inchesToMeters(24)) {
          m_currentZone = SUPERSTRUCTURE_STATE.EXTENDED_ZONE;
        }
        break;
      case LOW_ZONE:
        if (m_elevator.getHeightMeters() > Units.inchesToMeters(24)) {
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        }
        break;
      case STOWED:
        if (m_elevator.getHeightMeters() > Units.inchesToMeters(24)) {
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        }
        m_elevator.setSetpointState(ELEVATOR.SETPOINT.STOWED);
        m_wrist.setDesiredAngle(WRIST.SETPOINT.STOWED.get());
        break;
      case INTAKE_LOW:
        m_elevator.setSetpointState(ELEVATOR.SETPOINT.STOWED);
        m_wrist.setDesiredAngle(WRIST.SETPOINT.INTAKING_GROUND.get());
        // TODO: Determine Limit
        if (m_elevator.getHeightMeters() > ELEVATOR.THRESHOLD.LOW_TO_HIGH.get()) {
          m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
        }
        break;
      case INTAKE_EXTENDED:
        m_elevator.setSetpointState(ELEVATOR.SETPOINT.HIGH);
        m_wrist.setDesiredAngle(WRIST.SETPOINT.HIGH.get());
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
                    m_elevator.getElevatorTranslation(), m_drive.getHeadingRotation2d()));

    return targetPose.minus(elevatorPose).getTranslation().getNorm() > margin;
  }

  private void initSmartDashboard() {
    var stateHandlerTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("StateHandler");
    m_statePub = stateHandlerTab.getStringTopic("state").publish();
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
    SmartDashboard.putString("Superstructure State", getSuperStructureState().toString());
    //    SmartDashboard.putString("Superstructure State", getSuperStructureState().toString());
    m_statePub.set(getSuperStructureState().toString());
    m_elevatorHeightPub.set(Units.metersToInches(m_elevator.getHeightMeters()));
    m_elevatorUpperLimPub.set(Units.metersToInches(m_elevator.getUpperLimit()));
    m_elevatorLowerLimPub.set(Units.metersToInches(m_elevator.getLowerLimit()));
    m_wristAnglePub.set(m_wrist.getAngleDegrees());
    m_wristUpperLimPub.set(m_wrist.getUpperLimit());
    m_wristLowerLimPub.set(m_wrist.getLowerLimit());
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
          m_elevator.setUpperLimit(Units.inchesToMeters(43));
          m_elevator.setLowerLimit(Units.inchesToMeters(20));
          m_wrist.setUpperLimit(225);
          m_wrist.setLowerLimit(0);
          break;
        case HIGH_ZONE:
          m_elevator.setUpperLimit(Units.inchesToMeters(24));
          m_elevator.setLowerLimit(Units.inchesToMeters(0));
          m_wrist.setUpperLimit(80);
          m_wrist.setLowerLimit(0);
          break;
        case LOW_ZONE:
          m_elevator.setUpperLimit(Units.inchesToMeters(12));
          m_elevator.setLowerLimit(Units.inchesToMeters(0));
          m_wrist.setUpperLimit(80);
          m_wrist.setLowerLimit(-15);
        case STOWED:
          m_elevator.setUpperLimit(Units.inchesToMeters(12));
          m_elevator.setLowerLimit(Units.inchesToMeters(0));
          m_wrist.setUpperLimit(80);
          m_wrist.setLowerLimit(-15);
          break;
        case INTAKE_LOW:
          m_elevator.setUpperLimit(Units.inchesToMeters(12));
          m_elevator.setLowerLimit(Units.inchesToMeters(0));
          m_wrist.setUpperLimit(80);
          m_wrist.setLowerLimit(-15);
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
      m_wrist.setDesiredAngle(WRIST.SETPOINT.HIGH.get());
      m_elevator.setElevatorMotionMagicMeters(m_setpointSolver.getElevatorSetpointMeters());
      // TODO: Add this to the SwerveDrive
      // m_drive.setHeadingSetpoint(m_setpointSolver.getChassisSetpointRotation2d());
    }
  }
}
