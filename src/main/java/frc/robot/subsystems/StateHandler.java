// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.STATE_HANDLER.elevatorSetpointTolerance;
import static frc.robot.Constants.STATE_HANDLER.wristSetpointTolerance;
import static frc.robot.utils.ChargedUpNodeMask.getTargetNode;
import static frc.robot.utils.ChargedUpNodeMask.getValidNodes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.SCORING_STATE;
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.Constants.STATE_HANDLER.SUPERSTRUCTURE_STATE;
import frc.robot.Constants.STATE_HANDLER.ZONE;
import frc.robot.Constants.WRIST;
import frc.robot.commands.statehandler.SetSetpoint;
import frc.robot.utils.SetpointSolver;
import java.util.ArrayList;

public class StateHandler extends SubsystemBase implements AutoCloseable {
  /**
   * StateHandler Zones: Alpha, Beta, and Gamma Alpha is when elevator height is between 0-4 inches
   * Beta is when elevator height is between 3.5-28 inches Gamma is when elevator height is between
   * 27.5-50 inches
   */
  private SCORING_STATE m_scoringState = SCORING_STATE.STOWED;

  private double m_wristOffset = 0;
  private SUPERSTRUCTURE_STATE m_currentState = SUPERSTRUCTURE_STATE.STOWED;
  private SUPERSTRUCTURE_STATE m_currentDisplayedState = m_currentState;
  private SUPERSTRUCTURE_STATE m_lastState = m_currentState;
  private SUPERSTRUCTURE_STATE m_desiredState = m_currentState;
  private SETPOINT m_desiredSetpoint = SETPOINT.STOWED;
  private ZONE m_currentZone = ZONE.UNDEFINED;

  private final boolean m_limitCanUtil = STATE_HANDLER.limitCanUtilization;

  private boolean m_smartScoringEnabled;
  private boolean m_canScore;
  private boolean m_isOnTarget;

  private final Timer m_inactiveTimer = new Timer();
  private boolean inactiveTimerEnabled = false;
  private double timestamp;
  private double startTime = 0;

  private double m_elevatorDesiredSetpointMeters;
  private double m_wristDesiredSetpointRadians;
  private double elevatorLowerLimitMeters;
  private double elevatorUpperLimitMeters;
  private double wristLowerLimitRadians;
  private double wristUpperLimitRadians;
  private final double universalWristLowerLimitRadians =
      STATE_HANDLER.universalWristLowerLimitRadians;
  private final double universalWristUpperLimitRadians =
      STATE_HANDLER.universalWristUpperLimitRadians;

  private final Intake m_intake;
  private final Wrist m_wrist;
  private final SwerveDrive m_swerveDrive;
  private final Elevator m_elevator;
  private final Vision m_vision;
  private final SetpointSolver m_setpointSolver;
  private boolean m_isStateHandlerEnabled = true;

  public static final Mechanism2d m_superStructureMech2d =
      new Mechanism2d(STATE_HANDLER.mechanism2dOffset * 3, STATE_HANDLER.mechanism2dOffset * 3);
  public static final MechanismRoot2d m_chassisRoot2d =
      m_superStructureMech2d.getRoot(
          "ChassisRoot", STATE_HANDLER.mechanism2dOffset, STATE_HANDLER.mechanism2dOffset);
  public static final MechanismRoot2d m_elevatorRoot2d =
      m_superStructureMech2d.getRoot(
          "ElevatorRoot",
          STATE_HANDLER.mechanism2dOffset,
          STATE_HANDLER.mechanism2dOffset + Units.inchesToMeters(3));

  private static final Timer m_simTimer = new Timer();
  private static double m_lastSimTime;
  private static double m_currentSimTime;

  private final SendableChooser<SUPERSTRUCTURE_STATE> m_mainStateChooser = new SendableChooser<>();
  private final SendableChooser<SCORING_STATE> m_scoringStateChooser = new SendableChooser<>();
  private boolean m_testScoringState;

  private StringPublisher m_currentStatePub, m_desiredStatePub, m_currentZonePub;

  private BooleanPublisher m_isEnabledPub;
  private DoublePublisher m_elevatorHeightMetersPub,
      m_elevatorLowerLimitPub,
      m_elevatorUpperLimitPub,
      m_wristAnglePub,
      m_wristLowerLimitPub,
      m_wristUpperLimitPub;

  public StateHandler(
      Intake intake, Wrist wrist, SwerveDrive swerveDrive, Elevator elevator, Vision vision) {
    m_intake = intake;
    m_swerveDrive = swerveDrive;
    m_elevator = elevator;
    m_vision = vision;
    m_wrist = wrist;
    m_setpointSolver = SetpointSolver.getInstance();
    initSmartDashboard();

    m_inactiveTimer.reset();
    m_inactiveTimer.start();

    if (RobotBase.isSimulation()) {
      initializeScoringChooser();
      initializeMainStateChooser();
      // Attach the ligaments of the mech2d together
      try {
        m_elevator.getLigament().append(m_wrist.getLigament());
        m_wrist.getLigament().append(m_intake.getLigament());
      } catch (Exception e) {
        //        System.out.println("Ignoring WPILib Error");
      }
      SmartDashboard.putData("SuperStructure Sim", m_superStructureMech2d);
      m_simTimer.reset();
      m_simTimer.start();
    }
  }

  public void init() {
    m_currentState =
        determineSuperStructureState(m_elevator.getHeightMeters(), m_wrist.getPositionRadians());
  }

  public void initializeScoringChooser() {
    for (SCORING_STATE state : SCORING_STATE.values()) {
      m_scoringStateChooser.addOption(state.toString(), state);
    }

    m_scoringStateChooser.setDefaultOption("STOWED", SCORING_STATE.STOWED);

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

  public SUPERSTRUCTURE_STATE getCurrentDisplayedState() {
    return m_currentDisplayedState;
  }

  // returns the desired zone or state
  public SUPERSTRUCTURE_STATE getDesiredState() {
    return m_desiredState;
  }

  // returns the current zone transition
  public ZONE getCurrentZone() {
    return m_currentZone;
  }

  public boolean isScoring() {
    return getDesiredSetpoint() != SETPOINT.SCORE_MID_CONE
        || getDesiredSetpoint() != SETPOINT.SCORE_MID_CUBE
        || getDesiredSetpoint() != SETPOINT.SCORE_HIGH_CONE
        || getDesiredSetpoint() != SETPOINT.SCORE_HIGH_CUBE;
  }

  public void setTestScoringState(boolean state) {
    m_testScoringState = state;
  }

  public boolean getTestScoringState() {
    return m_testScoringState;
  }

  public void updateScoringState() {
    if (m_testScoringState) {
      m_scoringState = m_scoringStateChooser.getSelected();
    } else {
      var intakeState = m_intake.getIntakeState();

      if (getDesiredState() == SUPERSTRUCTURE_STATE.SCORE_LOW_REVERSE)
        m_scoringState = SCORING_STATE.LOW_REVERSE;
      else if (getDesiredState() == SUPERSTRUCTURE_STATE.INTAKE_LOW_CONE
          || getDesiredState() == SUPERSTRUCTURE_STATE.INTAKE_LOW_CUBE)
        m_scoringState = SCORING_STATE.LOW;
      else if (getDesiredState() == SUPERSTRUCTURE_STATE.SCORE_MID_CONE
          || getDesiredState() == SUPERSTRUCTURE_STATE.SCORE_MID_CUBE) {
        if (intakeState == INTAKE_STATE.HOLDING_CONE) m_scoringState = SCORING_STATE.MID_CONE;
        else if (intakeState == INTAKE_STATE.INTAKING_CUBE) m_scoringState = SCORING_STATE.MID_CUBE;
      } else if (getDesiredState() == SUPERSTRUCTURE_STATE.SCORE_HIGH_CONE
          || getDesiredState() == SUPERSTRUCTURE_STATE.SCORE_HIGH_CUBE) {
        if (intakeState == INTAKE_STATE.HOLDING_CONE) m_scoringState = SCORING_STATE.HIGH_CONE;
        else if (intakeState == INTAKE_STATE.INTAKING_CUBE)
          m_scoringState = SCORING_STATE.HIGH_CUBE;
      } else {
        m_scoringState = SCORING_STATE.STOWED;
      }
    }
  }

  public SCORING_STATE getScoringState() {
    return m_scoringState;
  }

  public boolean isSmartScoring() {
    return m_smartScoringEnabled;
  }

  public void setSmartScoring(boolean enabled) {
    m_smartScoringEnabled = enabled;
  }

  public boolean canScore() {
    return m_canScore;
  }

  public boolean isOnTarget() {
    return m_isOnTarget;
  }

  public void enable() {
    m_isStateHandlerEnabled = true;
  }

  public void disable() {
    m_isStateHandlerEnabled = false;
  }

  public boolean getIsStateHandlerEnabled() {
    return m_isStateHandlerEnabled;
  }

  public static double getCurrentSimTime() {
    return m_currentSimTime;
  }

  public static double getLastSimTime() {
    return m_lastSimTime;
  }

  public static double getSimDt() {
    return getCurrentSimTime() - getLastSimTime();
  }

  // Sets desired setpoint from setpoint enums created, clamps the setpoints before settings based
  // on local limits which are based on the current zone
  public void setDesiredSetpoint(STATE_HANDLER.SETPOINT desiredSetpoint) {
    m_desiredSetpoint = desiredSetpoint;
    SetElevatorDesiredSetpoint(desiredSetpoint);
    SetWristDesiredSetpoint(desiredSetpoint);
  }

  public void SetWristDesiredSetpoint(STATE_HANDLER.SETPOINT desiredSetpoint) {
    m_wristDesiredSetpointRadians = desiredSetpoint.getWristSetpointRadians();
  }

  public void SetElevatorDesiredSetpoint(STATE_HANDLER.SETPOINT desiredSetpoint) {
    m_elevatorDesiredSetpointMeters = desiredSetpoint.getElevatorSetpointMeters();
  }

  public STATE_HANDLER.SETPOINT getDesiredSetpoint() {
    return m_desiredSetpoint;
  }

  private void updateCommandedSetpoints() {
    if (m_isStateHandlerEnabled) {
      setElevatorCommandedSetpoint();
      setWristCommandedSetpoint();
    }
  }

  private void setElevatorCommandedSetpoint() {
    if ((m_currentState.getZone() == m_desiredState.getZone())
        || (m_wrist.getPositionRadians() >= universalWristLowerLimitRadians
            && m_wrist.getPositionRadians() <= universalWristUpperLimitRadians)) {
      m_elevator.setDesiredPositionMeters(
          MathUtil.clamp(
              m_elevatorDesiredSetpointMeters,
              ELEVATOR.THRESHOLD.ABSOLUTE_MIN.get(),
              ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get()));
    }
  }

  private void setWristCommandedSetpoint() {
    m_wrist.setSetpointPositionRadians(
        MathUtil.clamp(
            m_wristDesiredSetpointRadians, wristLowerLimitRadians, wristUpperLimitRadians));
  }

  // Determines the current state based off current wrist/elevator positions.
  public SUPERSTRUCTURE_STATE determineSuperStructureState(
      double elevatorPositionMeters, double wristPositionRadians) {
    SUPERSTRUCTURE_STATE assumedZone = SUPERSTRUCTURE_STATE.DANGER_ZONE;

    // Specific states defined by elevator/wrist setpoints
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.STOWED.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.STOWED.get()) < wristSetpointTolerance)
      return SUPERSTRUCTURE_STATE.STOWED;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.INTAKING_LOW.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.INTAKING_LOW_CONE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.INTAKE_LOW_CONE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.INTAKING_LOW.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.INTAKING_LOW_CUBE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.INTAKE_LOW_CUBE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_LOW_REVERSE.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_LOW_REVERSE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.SCORE_LOW_REVERSE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.INTAKING_EXTENDED_CONE.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.INTAKING_EXTENDED_CONE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.INTAKE_EXTENDED;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_LOW_CONE.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_LOW_CONE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.SCORE_LOW_CONE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_LOW_CUBE.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_LOW_CUBE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.SCORE_LOW_CUBE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_MID_CONE.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_MID_CONE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.SCORE_MID_CONE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_MID_CUBE.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_MID_CUBE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.SCORE_MID_CUBE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_HIGH_CONE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.SCORE_HIGH_CONE;
    if (Math.abs(elevatorPositionMeters - ELEVATOR.SETPOINT.SCORE_HIGH_CUBE.get())
            < elevatorSetpointTolerance
        && Math.abs(wristPositionRadians - WRIST.SETPOINT.SCORE_HIGH_CUBE.get())
            < wristSetpointTolerance) return SUPERSTRUCTURE_STATE.SCORE_HIGH_CUBE;

    // General states (zones) defined by region
    if (elevatorPositionMeters <= ELEVATOR.THRESHOLD.ALPHA_MAX.get()) {
      if (WRIST.THRESHOLD.ALPHA_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.ALPHA_MAX.get())
        return SUPERSTRUCTURE_STATE.ALPHA_ZONE;
      else assumedZone = SUPERSTRUCTURE_STATE.ALPHA_ZONE;
    }
    if (ELEVATOR.THRESHOLD.BETA_MIN.get() < elevatorPositionMeters
        && elevatorPositionMeters < ELEVATOR.THRESHOLD.BETA_MAX.get()) {
      if (WRIST.THRESHOLD.BETA_MIN.get() < wristPositionRadians
          && wristPositionRadians <= WRIST.THRESHOLD.BETA_MAX.get())
        return SUPERSTRUCTURE_STATE.BETA_ZONE;
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
    // Check current mechanism positions before advancing zones
    switch (m_currentState.getZone()) {
      case ALPHA:
        // ALPHA -> BETA
        if (ELEVATOR.THRESHOLD.BETA_MIN.get() < m_elevator.getHeightMeters()) {
          if (WRIST.THRESHOLD.BETA_MIN.get() < m_wrist.getPositionRadians()
              && m_wrist.getPositionRadians() < WRIST.THRESHOLD.BETA_MAX.get()) {
            m_currentState = SUPERSTRUCTURE_STATE.BETA_ZONE;
            m_currentZone = ZONE.BETA;
            return;
          }
        }
        break;

      case BETA:
        // BETA -> ALPHA
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.ALPHA_MAX.get()) {
          if (WRIST.THRESHOLD.ALPHA_MIN.get() < m_wrist.getPositionRadians()
              && m_wrist.getPositionRadians() < WRIST.THRESHOLD.ALPHA_MAX.get()) {
            m_currentState = SUPERSTRUCTURE_STATE.ALPHA_ZONE;
            m_currentZone = ZONE.ALPHA;
            return;
          }
        } else if (ELEVATOR.THRESHOLD.GAMMA_MIN.get() < m_elevator.getHeightMeters()) {
          if (WRIST.THRESHOLD.GAMMA_MIN.get() < m_wrist.getPositionRadians()
              && m_wrist.getPositionRadians() < WRIST.THRESHOLD.GAMMA_MAX.get()) {
            // BETA -> GAMMA
            m_currentState = SUPERSTRUCTURE_STATE.GAMMA_ZONE;
            m_currentZone = ZONE.GAMMA;
            return;
          }
        }
        break;
      case GAMMA:
        // GAMMA -> BETA
        if (m_elevator.getHeightMeters() < ELEVATOR.THRESHOLD.BETA_MAX.get()) {
          if (WRIST.THRESHOLD.BETA_MIN.get() < m_wrist.getPositionRadians()
              && m_wrist.getPositionRadians() < WRIST.THRESHOLD.BETA_MAX.get()) {
            m_currentState = SUPERSTRUCTURE_STATE.BETA_ZONE;
            m_currentZone = ZONE.BETA;
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

  public void updateZoneLimits() {
    switch (m_currentState.getZone()) {
      case ALPHA:
        setElevatorLowerLimitMeters(ELEVATOR.THRESHOLD.ALPHA_MIN.get());
        setElevatorUpperLimitMeters(ELEVATOR.THRESHOLD.ALPHA_MAX.get());
        setWristLowerLimitRadians(WRIST.THRESHOLD.ALPHA_MIN.get());
        setWristUpperLimitRadians(WRIST.THRESHOLD.ALPHA_MAX.get());
        break;
      case BETA:
        setElevatorLowerLimitMeters(ELEVATOR.THRESHOLD.BETA_MIN.get());
        setElevatorUpperLimitMeters(ELEVATOR.THRESHOLD.BETA_MAX.get());
        setWristLowerLimitRadians(WRIST.THRESHOLD.BETA_MIN.get());
        setWristUpperLimitRadians(WRIST.THRESHOLD.BETA_MAX.get());
        break;
      case GAMMA:
        setElevatorLowerLimitMeters(ELEVATOR.THRESHOLD.GAMMA_MIN.get());
        setElevatorUpperLimitMeters(ELEVATOR.THRESHOLD.GAMMA_MAX.get());
        setWristLowerLimitRadians(WRIST.THRESHOLD.GAMMA_MIN.get());
        setWristUpperLimitRadians(WRIST.THRESHOLD.GAMMA_MAX.get());
        break;
      default:
        // Undefined state, put a breakpoint here when debugging to check logic
        System.out.println("This should never be reached");
        break;
    }
    // If the desired state is not in the current zone, set the limits to the universal limits
    if (m_currentState.getZone() != m_desiredState.getZone()) {
      wristLowerLimitRadians = universalWristLowerLimitRadians;
      wristUpperLimitRadians = universalWristUpperLimitRadians;
    }
  }

  public boolean isRobotOnTarget(Pose2d targetPose, double margin) {
    var elevatorPose =
        m_swerveDrive
            .getPoseMeters()
            .transformBy(
                new Transform2d(
                    m_elevator.getField2dTranslation().plus(m_wrist.getHorizontalTranslation()),
                    m_swerveDrive.getHeadingRotation2d()));

    return targetPose.minus(elevatorPose).getTranslation().getNorm() > margin;
  }

  private void initSmartDashboard() {
    var stateHandlerTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("StateHandler");

    try {
      stateHandlerTab.getBooleanTopic("limitCANUtilization").publish().set(m_limitCanUtil);
    } catch (Exception m_ignored) {

    }

    m_isEnabledPub = stateHandlerTab.getBooleanTopic("isEnabled").publish();
    m_currentStatePub = stateHandlerTab.getStringTopic("currentState").publish();
    m_desiredStatePub = stateHandlerTab.getStringTopic("desiredState").publish();
    m_currentZonePub = stateHandlerTab.getStringTopic("currentZone").publish();
    m_elevatorHeightMetersPub = stateHandlerTab.getDoubleTopic("elevatorHeightInches").publish();
    m_elevatorLowerLimitPub = stateHandlerTab.getDoubleTopic("elevatorMinLimit").publish();
    m_elevatorUpperLimitPub = stateHandlerTab.getDoubleTopic("elevatorMaxLimit").publish();
    m_wristAnglePub = stateHandlerTab.getDoubleTopic("wristAngleDegrees").publish();
    m_wristLowerLimitPub = stateHandlerTab.getDoubleTopic("wristMinLimit").publish();
    m_wristUpperLimitPub = stateHandlerTab.getDoubleTopic("wristMaxLimit").publish();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putString("Superstructure State", getCurrentState().toString());
    m_isEnabledPub.set(getIsStateHandlerEnabled());
    m_currentStatePub.set(getCurrentDisplayedState().toString());
    m_desiredStatePub.set(getDesiredState().toString());
    m_currentZonePub.set(getCurrentZone().toString());
    m_elevatorHeightMetersPub.set(Units.metersToInches(m_elevator.getHeightMeters()));
    m_wristAnglePub.set(m_wrist.getPositionDegrees());

    if (!m_limitCanUtil) {
      m_elevatorUpperLimitPub.set(Units.metersToInches(elevatorUpperLimitMeters));
      m_elevatorLowerLimitPub.set(Units.metersToInches(elevatorLowerLimitMeters));
      m_wristUpperLimitPub.set(Units.radiansToDegrees(wristUpperLimitRadians));
      m_wristLowerLimitPub.set(Units.radiansToDegrees(wristLowerLimitRadians));
    }
  }

  // TODO: Fix this
  public void switchTargetNode(boolean left) {
    ArrayList<Translation2d> possibleNodes = getValidNodes();
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    updateZoneLimits();
    updateCommandedSetpoints();

    // Undefined behavior, use previous zone as a backup
    if (m_currentState.getZone() == SUPERSTRUCTURE_STATE.DANGER_ZONE.getZone()) {
      m_currentState = m_lastState;
    } else {
      m_lastState = m_currentState;
    }

    // Displayed state for easier debugging
    m_currentDisplayedState =
        determineSuperStructureState(m_elevator.getHeightMeters(), m_wrist.getPositionRadians());

    // Determine desired zone based on elevator/wrist setpoints
    m_desiredState =
        determineSuperStructureState(
            m_elevatorDesiredSetpointMeters, m_wristDesiredSetpointRadians);

    // Limit wrist/elevator setpoints to safe thresholds based on where you are and where you want
    // to go
    zoneAdvancement();

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

    if (m_intake.getRetractIntake()) {
      var retractCmd = new SetSetpoint(this, m_elevator, m_wrist, SETPOINT.STOWED);
      retractCmd.schedule();
      m_intake.setRetractIntake(false);
    }

    if (m_smartScoringEnabled) {
      updateScoringState();

      var targetNode = getTargetNode(m_swerveDrive.getPoseMeters());
      m_isOnTarget = isRobotOnTarget(targetNode, Units.inchesToMeters(1));
      m_wristOffset = m_wrist.getHorizontalTranslation().getX();
      m_setpointSolver.solveSetpoints(
          m_swerveDrive.getPoseMeters(), targetNode, m_wristOffset, getScoringState());
      m_canScore = m_setpointSolver.canScore();
      m_wrist.setSetpointPositionRadians(WRIST.SETPOINT.SCORE_HIGH_CONE.get());
      m_elevator.setDesiredPositionMeters(m_setpointSolver.getElevatorSetpointMeters());
      // TODO: Add this to the SwerveDrive
      // m_drive.setHeadingSetpoint(m_setpointSolver.getChassisSetpointRotation2d());
    }
  }

  private void setElevatorLowerLimitMeters(double lowerLimitMeters) {
    elevatorLowerLimitMeters = lowerLimitMeters;
  }

  private void setElevatorUpperLimitMeters(double upperLimitMeters) {
    elevatorUpperLimitMeters = upperLimitMeters;
  }

  private void setWristLowerLimitRadians(double lowerLimitRadians) {
    wristLowerLimitRadians = lowerLimitRadians;
  }

  private void setWristUpperLimitRadians(double upperLimitRadians) {
    wristUpperLimitRadians = upperLimitRadians;
  }

  public void testPeriodic() {
    m_currentState = m_mainStateChooser.getSelected();
  }

  @Override
  public void simulationPeriodic() {
    m_lastSimTime = m_currentSimTime;
    m_currentSimTime = m_simTimer.get();

    // This will fail unit tests for some reason
    try {
      // Update the angle of the mech2d
      m_elevator.getLigament().setLength(m_elevator.getHeightMeters() + ELEVATOR.carriageOffset);
      m_wrist
          .getLigament()
          .setAngle(180 - m_elevator.getLigament().getAngle() - m_wrist.getPositionDegrees());
      m_intake.getLigament().setAngle(m_wrist.getLigament().getAngle() * -1.5);
    } catch (Exception ignored) {

    }
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    m_superStructureMech2d.close();
    m_chassisRoot2d.close();
    m_elevatorRoot2d.close();
  }
}
