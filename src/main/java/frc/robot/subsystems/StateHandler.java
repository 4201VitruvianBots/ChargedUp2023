// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.STATEHANDLER.elevatorSetpointTolerance;
import static frc.robot.Constants.STATEHANDLER.wristSetpointTolerance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_UTIL_LIMIT;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.Constants.STATEHANDLER;
import frc.robot.Constants.STATEHANDLER.*;
import frc.robot.Constants.WRIST;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Wrist.WRIST_SPEED;
import frc.robot.utils.SetpointSolver;
import java.util.ArrayList;

public class StateHandler extends SubsystemBase implements AutoCloseable {
  /**
   * StateHandler Zones: Alpha, Beta, and Gamma Alpha is when elevator height is between 0-4 inches
   * Beta is when elevator height is between 3.5-28 inches Gamma is when elevator height is between
   * 27.5-50 inches
   */
  public SCORING_STATE m_scoringState = SCORING_STATE.STOWED;

  public INTAKING_STATES currentIntakeState = INTAKING_STATES.NONE;
  private double m_wristOffset = 0;
  public SUPERSTRUCTURE_STATE m_currentState = SUPERSTRUCTURE_STATE.STOWED;
  public SUPERSTRUCTURE_STATE m_lastState = m_currentState;
  public SUPERSTRUCTURE_STATE m_desiredState = m_currentState;
  public ZONE m_currentZone = ZONE.UNDEFINED;
  public ZONE m_nextZone = m_currentZone;
  public CAN_UTIL_LIMIT limitCanUtil = CAN_UTIL_LIMIT.LIMITED;
  public Pose2d targetNode;
  private boolean m_zoneEnforcement = true;
  private boolean m_smartScoringEnabled;
  private boolean m_isOnTarget;

  private final Timer m_inactiveTimer = new Timer();
  private boolean inactiveTimerEnabled = false;
  private double timestamp;
  private double elevatorLowerLimitMeters;
  private double elevatorUpperLimitMeters;
  private double wristLowerLimitRadians;
  private double wristUpperLimitRadians;

  private final Intake m_intake;
  private final Wrist m_wrist;
  private final SwerveDrive m_drive;
  private final FieldSim m_fieldSim;
  private final Elevator m_elevator;
  private final LEDSubsystem m_led;
  private final Vision m_vision;
  private final SetpointSolver m_setpointSolver;

  private final SendableChooser<SUPERSTRUCTURE_STATE> m_mainStateChooser = new SendableChooser<>();
  private final SendableChooser<Constants.SCORING_STATE> m_scoringStateChooser =
      new SendableChooser<>();

  private StringPublisher m_currentStatePub, m_desiredStatePub, m_currentZonePub, m_limitCanPub;
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
      LEDSubsystem led,
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

    m_inactiveTimer.reset();
    m_inactiveTimer.start();

    if (RobotBase.isSimulation()) {
      initializeScoringChooser();
      initializeMainStateChooser();
    }
  }

  public void init() {
    m_currentState =
        determineSuperStructureState(m_elevator.getHeightMeters(), m_wrist.getPositionRadians());
  }

  public void initializeScoringChooser() {
    for (Constants.SCORING_STATE state : Constants.SCORING_STATE.values()) {
      m_scoringStateChooser.addOption(state.toString(), state);
    }

    m_scoringStateChooser.setDefaultOption("STOWED", Constants.SCORING_STATE.STOWED);

    SmartDashboard.putData("Scoring State Selector", m_scoringStateChooser);
  }

  public void initializeMainStateChooser() {
    for (SUPERSTRUCTURE_STATE state : SUPERSTRUCTURE_STATE.values()) {
      m_mainStateChooser.addOption(state.toString(), state);
    }
    m_mainStateChooser.setDefaultOption("STOWED", SUPERSTRUCTURE_STATE.STOWED);

    SmartDashboard.putData("Main State Selector", m_mainStateChooser);
  }
  // returns current zone which could be an actual zone (alpha, beta, gamma) or a state (score
  // high/mid/low/intaking)
  public SUPERSTRUCTURE_STATE getCurrentState() {
    return m_currentState;
  }
  // returns the desired zone or state
  public SUPERSTRUCTURE_STATE getDesiredState() {
    return m_desiredState;
  }
  // returns the current zone transition
  public ZONE getCurrentZone() {
    return m_currentZone;
  }

  public SCORING_STATE getCurrentScoringState() {
    return m_scoringState;
  }

  public void setCurrentScoringState(SCORING_STATE state) {
    m_scoringState = state;
  }

  // TODO: Make this a constant under Constants.STATE_HANDLER
  public void setReduceCanUtilization(CAN_UTIL_LIMIT limitCan) {
    limitCanUtil = limitCan;
  }

  public CAN_UTIL_LIMIT getReduceCanUtilization() {
    return limitCanUtil;
  }

  public void enableSmartScoring(boolean enabled) {
    m_smartScoringEnabled = enabled;
  }

  public boolean isOnTarget() {
    return m_isOnTarget;
  }
  // Determines the current state based off current wrist/elevator positions.
  public SUPERSTRUCTURE_STATE determineSuperStructureState(
      double elevatorPositionMeters, double wristPositionRadians) {
    SUPERSTRUCTURE_STATE assumedZone = SUPERSTRUCTURE_STATE.DANGER_ZONE;

    // TODO: Update all of these comparisions with constants
    // Specific states defined by elevator/wrist setpoints
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.STOWED.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.STOWED.get()) < wristSetpointTolerance)
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
    if (elevatorPositionMeters <= ELEVATOR.THRESHOLD.ALPHA_MAX.get()) {
      if (WRIST.THRESHOLD.ALPHA_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.ALPHA_MAX.get())
        return SUPERSTRUCTURE_STATE.ALPHA_ZONE;
      else assumedZone = SUPERSTRUCTURE_STATE.ALPHA_ZONE;
    }
    if (ELEVATOR.THRESHOLD.BETA_MIN.get() < elevatorPositionMeters
        && elevatorPositionMeters <= ELEVATOR.THRESHOLD.BETA_MAX.get()) {
      if (WRIST.THRESHOLD.BETA_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.BETA_MAX.get())
        return SUPERSTRUCTURE_STATE.GAMMA_ZONE;
      else assumedZone = SUPERSTRUCTURE_STATE.BETA_ZONE;
    }
    if (ELEVATOR.THRESHOLD.GAMMA_MIN.get() < elevatorPositionMeters
        && elevatorPositionMeters <= ELEVATOR.THRESHOLD.GAMMA_MAX.get()) {
      if (WRIST.THRESHOLD.GAMMA_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.GAMMA_MAX.get())
        return SUPERSTRUCTURE_STATE.GAMMA_ZONE;
      else assumedZone = SUPERSTRUCTURE_STATE.GAMMA_ZONE;
    }

    // Undefined state, put a breakpoint here when debugging to check logic
    if (assumedZone == SUPERSTRUCTURE_STATE.DANGER_ZONE)
      System.out.println("This should never be reached");

    return assumedZone;
  }
  // Sets a zone transition based on ordinals of alpha, beta, gamma zones (1,2,3)
  public void zoneAdvancement() {
    // If your current zone is not equal to your desired zone, assume you are transitioning between
    // zones, otherwise don't set a transition limit
    //    if (m_currentState.getZone() != m_desiredState.getZone()) {
    //      // SpecialCases
    //
    //      switch (m_currentState.getZone()) {
    //        case 1: // ALPHA
    //          if (m_desiredState.getZone() > m_currentState.getZone())
    //            m_nextZone = ZONE_TRANSITIONS.ALPHA_TO_BETA;
    //          break;
    //        case 2: // BETA
    //          if (m_desiredState.getZone() < m_currentState.getZone())
    //            m_nextZone = ZONE_TRANSITIONS.BETA_TO_ALPHA;
    //          else if (m_desiredState.getZone() > m_currentState.getZone())
    //            m_nextZone = ZONE_TRANSITIONS.BETA_TO_GAMMA;
    //          break;
    //          case 3: // GAMMA
    //          if (m_desiredState.getZone() < m_currentState.getZone())
    //            m_nextZone = ZONE_TRANSITIONS.GAMMA_TO_BETA;
    //          break;
    //        case 0: // UNKNOWN
    //        default:
    //          // Undefined behavior, put a breakpoint here when debugging to check logic
    //          System.out.println("This should never be reached");
    //          break;
    //      }
    //    } else m_nextZone = ZONE_TRANSITIONS.NONE;

    // By adding this, we achieved the behavior that we want, the buttons are able to hold their
    // setpoints (score, mid, low),
    // and we're able to make all of your transitions cleanly

    // Use zone transition info to set mechanism limits. Only threshold limit when within
    // transition zones
    //    switch (m_nextZone) {
    //      case ALPHA_TO_BETA:
    //        if (ELEVATOR.THRESHOLD.BETA_MIN.get() < m_elevator.getHeightMeters()) {
    //          elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.BETA_MIN.get();
    //          elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.BETA_MAX.get();
    //          wristLowerLimitRadians = WRIST.THRESHOLD.BETA_MIN.get();
    //          wristUpperLimitRadians = WRIST.THRESHOLD.BETA_MAX.get();
    //          break;
    //        }
    //      case BETA_TO_ALPHA:
    //        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.ALPHA_MAX.get()) {
    //          elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.ALPHA_MIN.get();
    //          elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.BETA_MAX.get();
    //          wristLowerLimitRadians = WRIST.THRESHOLD.ALPHA_MIN.get();
    //          wristUpperLimitRadians = WRIST.THRESHOLD.ALPHA_MAX.get();
    //          break;
    //        }
    //      case BETA_TO_GAMMA:
    //        if (ELEVATOR.THRESHOLD.GAMMA_MIN.get() < m_elevator.getHeightMeters()) {
    //          elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.BETA_MIN.get();
    //          elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.GAMMA_MAX.get();
    //          wristLowerLimitRadians = WRIST.THRESHOLD.GAMMA_MIN.get();
    //          wristUpperLimitRadians = WRIST.THRESHOLD.BETA_MAX.get();
    //          break;
    //        }
    //      case GAMMA_TO_BETA:
    //        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.BETA_MAX.get()) {
    //          elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.BETA_MIN.get();
    //          elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.GAMMA_MAX.get();
    //          wristLowerLimitRadians = WRIST.THRESHOLD.GAMMA_MIN.get();
    //          // Modified to avoid wrist hitting elevator
    //          //          m_wrist.setUpperLimit(WRIST.THRESHOLD.HIGH_MAX.get());
    //          wristUpperLimitRadians = WRIST.THRESHOLD.BETA_MAX.get();
    //          break;
    //        }
    //      case GAMMA_TO_ALPHA:
    //        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.GAMMA_MIN.get()) {
    //          elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.ALPHA_MIN.get();
    //          elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.GAMMA_MAX.get();
    //          wristLowerLimitRadians = WRIST.THRESHOLD.ALPHA_MIN.get();
    //          wristUpperLimitRadians = WRIST.THRESHOLD.BETA_MAX.get();
    //          break;
    //        }
    //      default:
    //      case NONE:
    //        // If not in a transition zone, use the current zone's limits
    //        switch (m_currentState.getZone()) {
    //          case 0: // ALPHA
    //            elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.ALPHA_MIN.get();
    //            elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.ALPHA_MAX.get();
    //            wristLowerLimitRadians = WRIST.THRESHOLD.ALPHA_MIN.get();
    //            wristUpperLimitRadians = WRIST.THRESHOLD.ALPHA_MAX.get();
    //            break;
    //          case 1: // BETA
    //            elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.BETA_MIN.get();
    //            elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.BETA_MAX.get();
    //            wristLowerLimitRadians = WRIST.THRESHOLD.BETA_MIN.get();
    //            wristUpperLimitRadians = WRIST.THRESHOLD.BETA_MAX.get();
    //            break;
    //          case 2: // GAMMA
    //            elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.GAMMA_MIN.get();
    //            elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.GAMMA_MAX.get();
    //            wristLowerLimitRadians = WRIST.THRESHOLD.GAMMA_MIN.get();
    //            wristUpperLimitRadians = WRIST.THRESHOLD.GAMMA_MAX.get();
    //            break;
    //          default:
    //            // Undefined state, put a breakpoint here when debugging to check logic
    //            System.out.println("This should never be reached");
    //            break;
    //        }
    //        break;
    //    }

    // Check current mechanism positions before advancing zones
    switch (m_currentState.getZone()) {
      case 1: // ALPHA
        // ALPHA -> BETA
        if (ELEVATOR.THRESHOLD.BETA_MIN.get() < m_elevator.getHeightMeters()) {
          if (WRIST.THRESHOLD.BETA_MIN.get() < m_wrist.getPositionRadians()
              && m_wrist.getPositionRadians() < WRIST.THRESHOLD.BETA_MAX.get()) {
            m_currentState = SUPERSTRUCTURE_STATE.BETA_ZONE;
            return;
          }
        }
        break;

      case 2: // BETA
        // BETA -> ALPHA
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.ALPHA_MAX.get()) {
          if (WRIST.THRESHOLD.BETA_MIN.get() < m_wrist.getPositionRadians()
              && m_wrist.getPositionRadians() < WRIST.THRESHOLD.BETA_MAX.get()) {
            m_currentState = SUPERSTRUCTURE_STATE.ALPHA_ZONE;
            return;
          }
        } else if (ELEVATOR.THRESHOLD.GAMMA_MIN.get() < m_elevator.getHeightMeters()) {
          // BETA -> GAMMA
          if (WRIST.THRESHOLD.BETA_MIN.get() < m_wrist.getPositionRadians()
              && m_wrist.getPositionRadians() < WRIST.THRESHOLD.BETA_MAX.get()) {
            m_currentState = SUPERSTRUCTURE_STATE.GAMMA_ZONE;
            return;
          }
        }
        break;
      case 3: // GAMMA
        // GAMMA -> BETA
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.BETA_MAX.get()) {
          if (WRIST.THRESHOLD.BETA_MIN.get() < m_wrist.getPositionRadians()
              && m_wrist.getPositionRadians() < WRIST.THRESHOLD.BETA_MAX.get()) {
            m_currentState = SUPERSTRUCTURE_STATE.BETA_ZONE;
            return;
          }
        }
        break;
      case 0: // UNDEFINED
      default:
        // Undefined behavior, put a breakpoint here when debugging to check logic
        System.out.println("This should never be reached");
        break;
    }
  }

  // Sets desired setpoint from setpoint enums created, clamps the setpoints before settings based
  // on local limits which are based on the current zone
  public void setDesiredSetpoint(STATEHANDLER.SETPOINT desiredState) {
    SetElevatorDesiredSetpoint(desiredState);
    SetWristDesiredSetpoint(desiredState);
  }

  public void SetWristDesiredSetpoint(STATEHANDLER.SETPOINT desiredState) {
    m_wrist.setDesiredPositionRadians(
            desiredState.getWristSetpointRadians());
  }

  public void SetElevatorDesiredSetpoint(STATEHANDLER.SETPOINT desiredState) {
    m_elevator.setDesiredPositionMeters(
            desiredState.getElevatorSetpointMeters());
  }

  public void updateZoneLimits() {
    switch (m_currentState.getZone()) {
      case 1: // ALPHA
        elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.ALPHA_MIN.get();
        elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.ALPHA_MAX.get();
        wristLowerLimitRadians = WRIST.THRESHOLD.ALPHA_MIN.get();
        wristUpperLimitRadians = WRIST.THRESHOLD.ALPHA_MAX.get();
        break;
      case 2: // BETA
        elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.BETA_MIN.get();
        elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.BETA_MAX.get();
        wristLowerLimitRadians = WRIST.THRESHOLD.BETA_MIN.get();
        wristUpperLimitRadians = WRIST.THRESHOLD.BETA_MAX.get();
        break;
      case 3: // GAMMA
        elevatorLowerLimitMeters = ELEVATOR.THRESHOLD.GAMMA_MIN.get();
        elevatorUpperLimitMeters = ELEVATOR.THRESHOLD.GAMMA_MAX.get();
        wristLowerLimitRadians = WRIST.THRESHOLD.GAMMA_MIN.get();
        wristUpperLimitRadians = WRIST.THRESHOLD.GAMMA_MAX.get();
        break;
      case 0:
      default:
        // Undefined state, put a breakpoint here when debugging to check logic
        System.out.println("This should never be reached");
        break;
    }
  }

  public boolean isRobotOnTarget(Pose2d targetPose, double margin) {
    var elevatorPose =
        m_drive
            .getPoseMeters()
            .transformBy(
                new Transform2d(
                    m_elevator.getField2dTranslation().plus(m_wrist.getHorizontalTranslation()),
                    m_drive.getHeadingRotation2d()));

    return targetPose.minus(elevatorPose).getTranslation().getNorm() > margin;
  }

  private void initSmartDashboard() {
    var stateHandlerTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("StateHandler");
    m_currentStatePub = stateHandlerTab.getStringTopic("currentState").publish();
    m_desiredStatePub = stateHandlerTab.getStringTopic("desiredState").publish();
    m_currentZonePub = stateHandlerTab.getStringTopic("currentZone").publish();
    m_elevatorHeightPub = stateHandlerTab.getDoubleTopic("elevatorHeightInches").publish();
    m_elevatorLowerLimPub = stateHandlerTab.getDoubleTopic("elevatorMinLimit").publish();
    m_elevatorUpperLimPub = stateHandlerTab.getDoubleTopic("elevatorMaxLimit").publish();
    m_wristAnglePub = stateHandlerTab.getDoubleTopic("wristAngleDegrees").publish();
    m_wristLowerLimPub = stateHandlerTab.getDoubleTopic("wristMinLimit").publish();
    m_wristUpperLimPub = stateHandlerTab.getDoubleTopic("wristMaxLimit").publish();
    m_limitCanPub = stateHandlerTab.getStringTopic("canUtilization").publish();
  }

  private void updateSmartDashboard(CAN_UTIL_LIMIT limitCan) {
    SmartDashboard.putString("Superstructure State", getCurrentState().toString());

    m_currentStatePub.set(getCurrentState().toString());
    m_desiredStatePub.set(getDesiredState().toString());
    m_currentZonePub.set(getCurrentZone().toString());
    m_elevatorHeightPub.set(Units.metersToInches(m_elevator.getHeightMeters()));
    m_wristAnglePub.set(m_wrist.getPositionDegrees());
    m_limitCanPub.set(limitCanUtil.name());

    switch (limitCan) {
      case NORMAL:
        m_elevatorUpperLimPub.set(Units.metersToInches(elevatorUpperLimitMeters));
        m_elevatorLowerLimPub.set(Units.metersToInches(elevatorLowerLimitMeters));
        m_wristUpperLimPub.set(Units.radiansToDegrees(wristUpperLimitRadians));
        m_wristLowerLimPub.set(Units.radiansToDegrees(wristLowerLimitRadians));
        break;
      default:
      case LIMITED:
        m_elevatorUpperLimPub.set(Units.metersToInches(elevatorUpperLimitMeters));
        m_elevatorLowerLimPub.set(Units.metersToInches(elevatorLowerLimitMeters));
        m_wristUpperLimPub.set(Units.radiansToDegrees(wristUpperLimitRadians));
        m_wristLowerLimPub.set(Units.radiansToDegrees(wristLowerLimitRadians));
        break;
    }
  }

  public void switchTargetNode(boolean left) {
    ArrayList<Translation2d> possibleNodes = m_fieldSim.getValidNodes();
    targetNode = m_fieldSim.getAdjacentNode(targetNode, possibleNodes, left);
  }

  @Override
  public void periodic() {
    updateSmartDashboard(limitCanUtil);
    updateZoneLimits();
    // targetNode = m_fieldSim.getTargetNode(currentIntakeState, scoringState);

    //     Determine current zone based on elevator/wrist position

    // Undefined behavior, use previous zone as a backup
    if (m_currentState.getZone() == SUPERSTRUCTURE_STATE.DANGER_ZONE.getZone()) {
      m_currentState = m_lastState;
    } else {
      m_lastState = m_currentState;
    }

    // Determine desired zone based on elevator/wrist setpoints
    m_desiredState =
        determineSuperStructureState(
            m_elevator.getDesiredPositionMeters(), m_wrist.getDesiredPositionRadians());

    // Limit wrist/elevator setpoints to safe thresholds based on where you are and where you want
    // to go
    if (m_zoneEnforcement) {
      zoneAdvancement();
    }

    // If no user input for more than one second, then reset elevator to stowed
    if (!DriverStation.isAutonomous()) {
      if ((!m_elevator.isUserControlled() && !m_wrist.isUserControlled())
          && !inactiveTimerEnabled) {
        inactiveTimerEnabled = true;
        timestamp = m_inactiveTimer.get();
      } else if (inactiveTimerEnabled
          && (m_elevator.isUserControlled() || m_wrist.isUserControlled())) {
        inactiveTimerEnabled = false;
        timestamp = 0;
      }
      if (inactiveTimerEnabled) {
        if (m_inactiveTimer.get() - timestamp > 1 && timestamp != 0) {
          setDesiredSetpoint(SETPOINT.STOWED);
        }
      }
    }

    // If the elevator is low, use the fast Wrist Trapezoid profile for faster intaking
    if (m_elevator.getHeightMeters() < Units.inchesToMeters(4.0)) {
      m_wrist.updateTrapezoidProfileConstraints(WRIST_SPEED.FAST);
    } else {
      m_wrist.updateTrapezoidProfileConstraints(WRIST_SPEED.SLOW);
    }

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
          m_fieldSim.getTargetNode(),
          m_wrist.getHorizontalTranslation().getX(),
          m_scoringState);
      m_wrist.setDesiredPositionRadians(WRIST.SETPOINT.SCORE_HIGH_CONE.get());
      m_elevator.setSetpointMotionMagicMeters(m_setpointSolver.getElevatorSetpointMeters());
      // TODO: Add this to the SwerveDrive
      // m_drive.setHeadingSetpoint(m_setpointSolver.getChassisSetpointRotation2d());
    }

    m_elevator.setReduceCanUtilization(limitCanUtil);
    m_wrist.setReduceCanUtilization(limitCanUtil);
    m_drive.setReduceCanUtilization(limitCanUtil);
  }

  public void setElevatorLowerLimitMeters(double lowerLimitMeters) {
    elevatorLowerLimitMeters = lowerLimitMeters;
  }

  public void setelevatorUpperLimitMetersMeters(double upperLimitMeters) {
    elevatorUpperLimitMeters = upperLimitMeters;
  }

  public void setwristLowerLimitRadians(double lowerLimitRadians) {
    wristLowerLimitRadians = lowerLimitRadians;
  }

  public void setwristUpperLimitRadians(double upperLimitRadians) {
    wristUpperLimitRadians = upperLimitRadians;
  }

  public void testPeriodic() {
    m_scoringState = m_scoringStateChooser.getSelected();
    m_currentState = m_mainStateChooser.getSelected();
    m_fieldSim.getTargetNode();
  }

  @Override
  public void close() throws Exception {}
}
