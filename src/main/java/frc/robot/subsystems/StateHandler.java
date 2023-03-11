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
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.Constants.WRIST;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Wrist.WRIST_SPEED;
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
    // UNDEFINED
    DANGER_ZONE(0),
    // LOW
    STOWED(1),
    INTAKE_LOW(1),
    SCORE_LOW_REVERSE(1),
    SCORE_LOW(1),
    SCORE_LOW_CONE(1),
    SCORE_LOW_CUBE(1),
    LOW_ZONE(1),
    // MID
    MID_ZONE(2),
    // HIGH
    HIGH_ZONE(3),
    // EXTENDED
    EXTENDED_ZONE(4),
    INTAKE_EXTENDED(4),
    SCORE_MID(4),
    SCORE_HIGH(4),
    SCORE_MID_CONE(4),
    SCORE_MID_CUBE(4),
    SCORE_HIGH_CONE(4),
    SCORE_HIGH_CUBE(4);

    // State Zone is determined by elevator setpoints
    private final int zone;

    SUPERSTRUCTURE_STATE(final int zone) {
      this.zone = zone;
    }

    public int getZone() {
      return zone;
    }
  }

  public enum ZONE_TRANSITIONS {
    NONE,
    LOW_TO_MID,
    MID_TO_LOW,
    MID_TO_HIGH,
    HIGH_TO_MID,
    HIGH_TO_EXTENDED,
    EXTENDED_TO_HIGH
  }

  public SCORING_STATE scoringState = SCORING_STATE.STOWED;
  public INTAKING_STATES currentIntakeState = INTAKING_STATES.NONE;
  private double m_wristOffset = 0;
  public SUPERSTRUCTURE_STATE m_currentZone = SUPERSTRUCTURE_STATE.STOWED;
  public SUPERSTRUCTURE_STATE m_lastZone = m_currentZone;
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

  private StringPublisher m_currentStatePub, m_desiredStatePub, m_nextZonePub;
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

  public ZONE_TRANSITIONS getNextZone() {
    return m_nextZone;
  }

  public void enableSmartScoring(boolean enabled) {
    m_smartScoringEnabled = enabled;
  }

  public boolean isOnTarget() {
    return m_isOnTarget;
  }

  public SUPERSTRUCTURE_STATE determineSuperStructureState(
      double elevatorPositionMeters, double wristPositionRadians) {
    SUPERSTRUCTURE_STATE assumedZone = SUPERSTRUCTURE_STATE.DANGER_ZONE;

    // Specific states defined by elevator/wrist setpoints
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.STOWED.get()) < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.STOWED.get()) < Units.degreesToRadians(4))
      return SUPERSTRUCTURE_STATE.STOWED;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.INTAKING_LOW.get())
            < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.INTAKING_LOW.get())
            < Units.degreesToRadians(4)) return SUPERSTRUCTURE_STATE.INTAKE_LOW;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_LOW_REVERSE.get())
            < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_LOW_REVERSE.get())
            < Units.degreesToRadians(4)) return SUPERSTRUCTURE_STATE.SCORE_LOW_REVERSE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.INTAKING_EXTENDED.get())
            < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.INTAKING_EXTENDED.get())
            < Units.degreesToRadians(4)) return SUPERSTRUCTURE_STATE.INTAKE_EXTENDED;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_LOW_CONE.get())
            < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_LOW_CONE.get())
            < Units.degreesToRadians(4)) return SUPERSTRUCTURE_STATE.SCORE_LOW_CONE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_LOW_CUBE.get())
            < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_LOW_CUBE.get())
            < Units.degreesToRadians(4)) return SUPERSTRUCTURE_STATE.SCORE_LOW_CUBE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_MID_CONE.get())
            < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_MID_CONE.get())
            < Units.degreesToRadians(4)) return SUPERSTRUCTURE_STATE.SCORE_MID_CONE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_MID_CUBE.get())
            < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_MID_CUBE.get())
            < Units.degreesToRadians(4)) return SUPERSTRUCTURE_STATE.SCORE_MID_CUBE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get())
            < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_HIGH_CONE.get())
            < Units.degreesToRadians(4)) return SUPERSTRUCTURE_STATE.SCORE_HIGH_CONE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_HIGH_CUBE.get())
            < Units.inchesToMeters(1)
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_HIGH_CUBE.get())
            < Units.degreesToRadians(4)) return SUPERSTRUCTURE_STATE.SCORE_HIGH_CUBE;

    // General states (zones) defined by region
    if (elevatorPositionMeters <= ELEVATOR.THRESHOLD.LOW_MAX.get()) {
      if (WRIST.THRESHOLD.LOW_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.LOW_MAX.get())
        return SUPERSTRUCTURE_STATE.LOW_ZONE;
      else assumedZone = SUPERSTRUCTURE_STATE.LOW_ZONE;
    }
    if (ELEVATOR.THRESHOLD.MID_MIN.get() < elevatorPositionMeters
        && elevatorPositionMeters <= ELEVATOR.THRESHOLD.MID_MAX.get()) {
      if (WRIST.THRESHOLD.MID_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.MID_MAX.get())
        return SUPERSTRUCTURE_STATE.MID_ZONE;
      else assumedZone = SUPERSTRUCTURE_STATE.MID_ZONE;
    }
    if (ELEVATOR.THRESHOLD.HIGH_MIN.get() < elevatorPositionMeters
        && elevatorPositionMeters <= ELEVATOR.THRESHOLD.HIGH_MAX.get()) {
      if (WRIST.THRESHOLD.HIGH_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.HIGH_MAX.get())
        return SUPERSTRUCTURE_STATE.HIGH_ZONE;
      else assumedZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
    }
    if (ELEVATOR.THRESHOLD.EXTENDED_MIN.get() < elevatorPositionMeters
        && elevatorPositionMeters <= ELEVATOR.THRESHOLD.EXTENDED_MAX.get()) {
      if (WRIST.THRESHOLD.EXTENDED_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.EXTENDED_MAX.get())
        return SUPERSTRUCTURE_STATE.EXTENDED_ZONE;
      else assumedZone = SUPERSTRUCTURE_STATE.EXTENDED_ZONE;
    }

    // Undefined state, put a breakpoint here when debugging to check logic
    if (assumedZone == SUPERSTRUCTURE_STATE.DANGER_ZONE)
      System.out.println("This should never be reached");

    return assumedZone;
  }

  public void zoneAdvancement() {
    // If your current zone is not equal to your desired zone, assume you are transitioning between
    // zones
    if(m_currentZone.getZone() != m_desiredZone.getZone()) {
      if (m_currentZone.getZone() == SUPERSTRUCTURE_STATE.LOW_ZONE.getZone()) {
        if (m_desiredZone.getZone() > m_currentZone.getZone())
          m_nextZone = ZONE_TRANSITIONS.LOW_TO_MID;
      } else if (m_currentZone.getZone() == SUPERSTRUCTURE_STATE.MID_ZONE.getZone()) {
        if (m_desiredZone.getZone() > m_currentZone.getZone())
          m_nextZone = ZONE_TRANSITIONS.MID_TO_HIGH;
        if (m_desiredZone.getZone() < m_currentZone.getZone())
          m_nextZone = ZONE_TRANSITIONS.MID_TO_LOW;
      } else if (m_currentZone.getZone() == SUPERSTRUCTURE_STATE.HIGH_ZONE.getZone()) {
        if (m_desiredZone.getZone() > m_currentZone.getZone())
          m_nextZone = ZONE_TRANSITIONS.HIGH_TO_EXTENDED;
        if (m_desiredZone.getZone() < m_currentZone.getZone())
          m_nextZone = ZONE_TRANSITIONS.HIGH_TO_MID;
      } else if (m_currentZone.getZone() == SUPERSTRUCTURE_STATE.EXTENDED_ZONE.getZone()) {
        if (m_desiredZone.getZone() < m_currentZone.getZone())
          m_nextZone = ZONE_TRANSITIONS.EXTENDED_TO_HIGH;
      } else {
        // Undefined behavior, put a breakpoint here when debugging to check logic
        System.out.println("This should never be reached");
      }
    } else m_nextZone = ZONE_TRANSITIONS.NONE;

    // Use zone transition info to set mechanism limits. Only threshold limit when within
    // transition
    // zones
    switch (m_nextZone) {
      case LOW_TO_MID:
        if (ELEVATOR.THRESHOLD.MID_MIN.get() < m_elevator.getHeightMeters()) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.LOW_MIN.get(), ELEVATOR.THRESHOLD.MID_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.LOW_MAX.get(), ELEVATOR.THRESHOLD.MID_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.LOW_MIN.get(), WRIST.THRESHOLD.MID_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.LOW_MAX.get(), WRIST.THRESHOLD.MID_MAX.get()));
        }
        break;
      case MID_TO_LOW:
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.LOW_MAX.get()) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.LOW_MIN.get(), ELEVATOR.THRESHOLD.MID_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.LOW_MAX.get(), ELEVATOR.THRESHOLD.MID_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.LOW_MIN.get(), WRIST.THRESHOLD.MID_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.LOW_MAX.get(), WRIST.THRESHOLD.MID_MAX.get()));
        }
        break;
      case MID_TO_HIGH:
        if (ELEVATOR.THRESHOLD.HIGH_MIN.get() < m_elevator.getHeightMeters()) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.MID_MIN.get(), ELEVATOR.THRESHOLD.HIGH_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.MID_MAX.get(), ELEVATOR.THRESHOLD.HIGH_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.MID_MIN.get(), WRIST.THRESHOLD.HIGH_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.MID_MAX.get(), WRIST.THRESHOLD.HIGH_MAX.get()));
        }
        break;
      case HIGH_TO_MID:
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.MID_MAX.get()) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.MID_MIN.get(), ELEVATOR.THRESHOLD.HIGH_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.MID_MAX.get(), ELEVATOR.THRESHOLD.HIGH_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.MID_MIN.get(), WRIST.THRESHOLD.HIGH_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.MID_MAX.get(), WRIST.THRESHOLD.HIGH_MAX.get()));
        }
        break;
      case HIGH_TO_EXTENDED:
        if (ELEVATOR.THRESHOLD.EXTENDED_MIN.get() < m_elevator.getHeightMeters()) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.HIGH_MIN.get(), ELEVATOR.THRESHOLD.EXTENDED_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.HIGH_MAX.get(), ELEVATOR.THRESHOLD.EXTENDED_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.HIGH_MIN.get(), WRIST.THRESHOLD.EXTENDED_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.HIGH_MAX.get(), WRIST.THRESHOLD.EXTENDED_MAX.get()));
        }
        break;
      case EXTENDED_TO_HIGH:
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.HIGH_MAX.get()) {
          m_elevator.setLowerLimitMeters(
              Math.max(ELEVATOR.THRESHOLD.HIGH_MIN.get(), ELEVATOR.THRESHOLD.EXTENDED_MIN.get()));
          m_elevator.setUpperLimitMeters(
              Math.min(ELEVATOR.THRESHOLD.HIGH_MAX.get(), ELEVATOR.THRESHOLD.EXTENDED_MAX.get()));
          m_wrist.setLowerLimit(
              Math.max(WRIST.THRESHOLD.HIGH_MIN.get(), WRIST.THRESHOLD.EXTENDED_MIN.get()));
          m_wrist.setUpperLimit(
              Math.min(WRIST.THRESHOLD.HIGH_MAX.get(), WRIST.THRESHOLD.EXTENDED_MAX.get()));
        }
        break;
      default:
      case NONE:
        switch (m_currentZone.getZone()) {
          case 1: // LOW
            m_elevator.setLowerLimitMeters(ELEVATOR.THRESHOLD.LOW_MIN.get());
            m_elevator.setUpperLimitMeters(ELEVATOR.THRESHOLD.LOW_MAX.get());
            m_wrist.setLowerLimit(WRIST.THRESHOLD.LOW_MIN.get());
            m_wrist.setUpperLimit(WRIST.THRESHOLD.LOW_MAX.get());
            break;
          case 2: // MID
            m_elevator.setLowerLimitMeters(ELEVATOR.THRESHOLD.MID_MIN.get());
            m_elevator.setUpperLimitMeters(ELEVATOR.THRESHOLD.MID_MAX.get());
            m_wrist.setLowerLimit(WRIST.THRESHOLD.MID_MIN.get());
            m_wrist.setUpperLimit(WRIST.THRESHOLD.MID_MAX.get());
            break;
          case 3: // HIGH
            m_elevator.setLowerLimitMeters(ELEVATOR.THRESHOLD.HIGH_MIN.get());
            m_elevator.setUpperLimitMeters(ELEVATOR.THRESHOLD.HIGH_MAX.get());
            m_wrist.setLowerLimit(WRIST.THRESHOLD.HIGH_MIN.get());
            m_wrist.setUpperLimit(WRIST.THRESHOLD.HIGH_MAX.get());
            break;
          case 4: // EXTENDED
            m_elevator.setLowerLimitMeters(ELEVATOR.THRESHOLD.EXTENDED_MIN.get());
            m_elevator.setUpperLimitMeters(ELEVATOR.THRESHOLD.EXTENDED_MAX.get());
            m_wrist.setLowerLimit(WRIST.THRESHOLD.EXTENDED_MIN.get());
            m_wrist.setUpperLimit(WRIST.THRESHOLD.EXTENDED_MAX.get());
            break;
          default:
            // Undefined state, put a breakpoint here when debugging to check logic
            System.out.println("This should never be reached");
            break;
        }
        break;
    }

    // Check current mechanism positions before advancing zones
    switch (m_currentZone.getZone()) {
      case 1: // LOW
        // LOW -> MIN
        if (ELEVATOR.THRESHOLD.MID_MIN.get() < m_elevator.getHeightMeters()) {
          if (WRIST.THRESHOLD.MID_MIN.get() < m_wrist.getPositionRadians()) {
            m_currentZone = SUPERSTRUCTURE_STATE.MID_ZONE;
            return;
          }
        }
        break;
      case 2: // MID
        // MID -> LOW
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.LOW_MAX.get()) {
          if (m_wrist.getPositionRadians() < WRIST.THRESHOLD.LOW_MAX.get()) {
            m_currentZone = SUPERSTRUCTURE_STATE.LOW_ZONE;
            return;
          }
        } else if (ELEVATOR.THRESHOLD.HIGH_MIN.get() < m_elevator.getHeightMeters()) {
          if (WRIST.THRESHOLD.HIGH_MIN.get() < m_wrist.getPositionRadians()) {
            m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
            return;
          }
        }
        break;
      case 3: // HIGH
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.MID_MAX.get()) {
          if (m_wrist.getPositionRadians() < WRIST.THRESHOLD.MID_MAX.get()) {
            m_currentZone = SUPERSTRUCTURE_STATE.MID_ZONE;
            return;
          }
        } else if (ELEVATOR.THRESHOLD.EXTENDED_MIN.get() < m_elevator.getHeightMeters()) {
          if (WRIST.THRESHOLD.EXTENDED_MIN.get() < m_wrist.getPositionRadians()) {
            m_currentZone = SUPERSTRUCTURE_STATE.EXTENDED_ZONE;
            return;
          }
        }
        break;
      case 4: // EXTENDED
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.HIGH_MAX.get()) {
          if (m_wrist.getPositionRadians() < WRIST.THRESHOLD.HIGH_MAX.get()) {
            m_currentZone = SUPERSTRUCTURE_STATE.HIGH_ZONE;
            return;
          }
        }
        break;
      default:
        // Undefined behavior, put a breakpoint here when debugging to check logic
        System.out.println("This should never be reached");
        break;
    }
  }

  private boolean isTipping() {
    return m_drive.getPitchDegrees() > 12.5;
  }

  // Whopper whopper whopper whopper
  // junior double triple whopper
  // flame grilled taste with perfect toppers
  // I rule this day
  public boolean isRobotOnTarget(Pose2d targetPose, double margin) {
    var elevatorPose =
        m_drive
            .getPoseMeters()
            .transformBy(
                new Transform2d(
                    m_elevator
                        .getElevatorField2dTranslation()
                        .plus(m_wrist.getHorizontalTranslation()),
                    m_drive.getHeadingRotation2d()));

    return targetPose.minus(elevatorPose).getTranslation().getNorm() > margin;
  }

  private void initSmartDashboard() {
    var stateHandlerTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("StateHandler");
    m_currentStatePub = stateHandlerTab.getStringTopic("currentState").publish();
    m_desiredStatePub = stateHandlerTab.getStringTopic("desiredState").publish();
    m_nextZonePub = stateHandlerTab.getStringTopic("nextZone").publish();
    m_elevatorHeightPub = stateHandlerTab.getDoubleTopic("elevatorHeightInches").publish();
    m_elevatorLowerLimPub = stateHandlerTab.getDoubleTopic("elevatorMinLimit").publish();
    m_elevatorUpperLimPub = stateHandlerTab.getDoubleTopic("elevatorMaxLimit").publish();
    m_wristAnglePub = stateHandlerTab.getDoubleTopic("wristAngleDegrees").publish();
    m_wristLowerLimPub = stateHandlerTab.getDoubleTopic("wristMinLimit").publish();
    m_wristUpperLimPub = stateHandlerTab.getDoubleTopic("wristMaxLimit").publish();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putString("Superstructure State", getCurrentZone().toString());

    m_currentStatePub.set(getCurrentZone().toString());
    m_desiredStatePub.set(getDesiredZone().toString());
    m_nextZonePub.set(getNextZone().toString());
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
    // targetNode = m_fieldSim.getTargetNode(currentIntakeState, scoringState);

    // Determine current zone based on elevator/wrist position
    m_currentZone =
        determineSuperStructureState(m_elevator.getHeightMeters(), m_wrist.getPositionRadians());

    if (m_currentZone.getZone() == SUPERSTRUCTURE_STATE.DANGER_ZONE.getZone()) {
      m_currentZone = m_lastZone;
    } else {
      m_lastZone = m_currentZone;
    }

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

    if (m_currentZone.getZone() == SUPERSTRUCTURE_STATE.LOW_ZONE.getZone()) {
      m_wrist.updateTrapezoidProfileConstraints(WRIST_SPEED.FAST);
    } else {
      m_wrist.updateTrapezoidProfileConstraints(WRIST_SPEED.SLOW);
    }

    // TODO: Limit max swerve speed by elevator height
    //    if (isTipping()) {
    //      m_elevator.setDesiredPositionMeters(ELEVATOR.SETPOINT.STOWED.get());
    //      m_wrist.setDesiredPositionRadians(WRIST.SETPOINT.STOWED.get());
    //    }

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
      m_isOnTarget = isRobotOnTarget(targetNode, 0.1);

      m_setpointSolver.solveSetpoints(
          m_drive.getPoseMeters(),
          m_fieldSim.getTargetNode(currentIntakeState, scoringState),
          m_wrist.getHorizontalTranslation().getX(),
          scoringState);
      m_wrist.setDesiredPositionRadians(WRIST.SETPOINT.SCORE_HIGH_CONE.get());
      m_elevator.setElevatorMotionMagicMeters(m_setpointSolver.getElevatorSetpointMeters());
      // TODO: Add this to the SwerveDrive
      // m_drive.setHeadingSetpoint(m_setpointSolver.getChassisSetpointRotation2d());
    }
  }
}
