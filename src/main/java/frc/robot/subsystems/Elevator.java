// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ELEVATOR.centerOffset;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_UTIL_LIMIT;
import frc.robot.Constants.ELEVATOR;

public class Elevator extends SubsystemBase implements AutoCloseable {

  // Initializing both motors
  public final TalonFX[] elevatorMotors = {
    new TalonFX(Constants.CAN.elevatorMotorLeft), new TalonFX(Constants.CAN.elevatorMotorRight)
  };

  // Initializing limit switch at bottom of elevator
  private final DigitalInput lowerLimitSwitch =
      new DigitalInput(Constants.DIO.elevatorLowerLimitSwitch);
  private boolean lowerLimitSwitchTriggered = false;

  private static double heightMeters =
      0; // the amount of meters the motor has moved up from the initial stowed position

  // Constants with value determined at runtime
  private final double maxHeightMeters = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get();
  private final int simEncoderSign =
      Constants.ELEVATOR.mainMotorInversionType == TalonFXInvertType.Clockwise ? -1 : 1;

  private double
      m_desiredPositionInputMeters; // The height in encoder units our robot is trying to reach
  private double
      m_desiredPositionOutputMeters; // The height in encoder units our robot is trying to reach
  private double
      m_commandedPositionMeters; // The height in encoder units our robot is trying to reach
  private ELEVATOR.SETPOINT m_desiredHeightState = ELEVATOR.SETPOINT.STOWED;
  private ELEVATOR.STATE m_controlState = ELEVATOR.STATE.AUTO_SETPOINT;
  private CAN_UTIL_LIMIT m_limitCanUtil = CAN_UTIL_LIMIT.NORMAL;

  // By default, this is set to true as we use the trapezoid profile to determine what speed we
  // should be at
  // to get to our setpoint.
  // If the sensors are acting up, we set this value false to directly control the percent output
  // of the motors.
  private boolean isClosedLoop = true;

  private boolean isElevatorElevatingElevatando = false;

  private double currentForwardOutput = 0;
  private double newForwardOutput = 0;

  // Positional limits set by the state handler
  private double m_lowerLimitMeters = ELEVATOR.THRESHOLD.ABSOLUTE_MIN.get();
  private double m_upperLimitMeters = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get();

  // Controlled by open loop
  private double m_joystickInput;
  private boolean m_userSetpoint;

  // Trapezoid profile setup
  private final TrapezoidProfile.Constraints m_stopSlippingConstraints =
      new TrapezoidProfile.Constraints(
          Constants.ELEVATOR.kMaxVel * .5, Constants.ELEVATOR.kMaxAccel);
  private final TrapezoidProfile.Constraints m_slowConstraints =
      new TrapezoidProfile.Constraints(Constants.ELEVATOR.kMaxVel, Constants.ELEVATOR.kMaxAccel);
  private final TrapezoidProfile.Constraints m_fastConstraints =
      new TrapezoidProfile.Constraints(
          Constants.ELEVATOR.kMaxVel * 1.3, Constants.ELEVATOR.kMaxAccel * 1.3);
  private TrapezoidProfile.Constraints m_currentConstraints = m_slowConstraints;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(
          Constants.ELEVATOR.kG, Constants.ELEVATOR.kV, Constants.ELEVATOR.kA);
  private final Timer m_timer = new Timer();
  private double m_lastTimestamp = 0;
  private double m_lastSimTimestamp = 0;

  // Simulation setup
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          Constants.ELEVATOR.gearbox,
          Constants.ELEVATOR.gearRatio,
          Constants.ELEVATOR.massKg,
          Constants.ELEVATOR.drumRadiusMeters,
          ELEVATOR.THRESHOLD.ABSOLUTE_MIN.get(),
          ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get(),
          true);

  // Shuffleboard setup
  private DoubleSubscriber kSetpointSub;
  private DoublePublisher kHeightPub,
      kEncoderCountsPub,
      kDesiredHeightPub,
      kHeightInchesPub,
      kPercentOutputPub;
  private StringPublisher kDesiredStatePub, kClosedLoopModePub, currentCommandStatePub;
  private BooleanPublisher lowerLimitSwitchPub;

  // Mechanism2d visualization setup
  public Mechanism2d mech2d = new Mechanism2d(maxHeightMeters * 50, maxHeightMeters * 50);
  public MechanismRoot2d root2d = mech2d.getRoot("Elevator", maxHeightMeters * 25, 0);
  public MechanismLigament2d elevatorLigament2d =
      root2d.append(new MechanismLigament2d("Elevator", heightMeters, 90));

  // Logging setup
  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry outputCurrentEntry = new DoubleLogEntry(log, "/elevator/current");
  public DoubleLogEntry setpointMetersEntry = new DoubleLogEntry(log, "/elevator/setpoint");
  public DoubleLogEntry positionMetersEntry = new DoubleLogEntry(log, "/elevator/position");

  /* Constructs a new Elevator. Mostly motor setup */
  public Elevator() {
    for (TalonFX motor : elevatorMotors) {
      motor.configFactoryDefault();
      motor.setNeutralMode(NeutralMode.Brake);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(0.0);

      // Config PID
      motor.selectProfileSlot(Constants.ELEVATOR.kSlotIdx, Constants.ELEVATOR.kPIDLoopIdx);
      motor.config_kP(
          Constants.ELEVATOR.kSlotIdx, Constants.ELEVATOR.kP, Constants.ELEVATOR.kTimeoutMs);
      motor.config_kI(
          Constants.ELEVATOR.kSlotIdx, Constants.ELEVATOR.kI, Constants.ELEVATOR.kTimeoutMs);
      motor.config_kD(
          Constants.ELEVATOR.kSlotIdx, Constants.ELEVATOR.kD, Constants.ELEVATOR.kTimeoutMs);

      motor.configPeakOutputForward(
          Constants.ELEVATOR.kMaxForwardOutput, Constants.ELEVATOR.kTimeoutMs);
      motor.configPeakOutputReverse(
          Constants.ELEVATOR.kMaxReverseOutput, Constants.ELEVATOR.kTimeoutMs);
      motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 0.1));
    }

    elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());

    elevatorMotors[0].setInverted(Constants.ELEVATOR.mainMotorInversionType);
    elevatorMotors[1].setInverted(TalonFXInvertType.OpposeMaster);
    elevatorMotors[1].setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    elevatorMotors[1].setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    initShuffleboard(m_limitCanUtil);
    m_timer.reset();
    m_timer.start();
  }

  // Elevator's motor output as a percentage
  public double getPercentOutput() {
    return elevatorMotors[0].getMotorOutputPercent();
  }

  public void setPercentOutput(double output) {
    if (getLimitSwitch() && output < 0) output = Math.max(output, 0);
    elevatorMotors[0].set(ControlMode.PercentOutput, output);
  }

  public void setSetpointMotionMagicMeters(double setpoint) {
    elevatorMotors[0].set(
        TalonFXControlMode.MotionMagic, setpoint / Constants.ELEVATOR.encoderCountsToMeters);
  }

  private TrapezoidProfile.State limitDesiredSetpointMeters(TrapezoidProfile.State state) {
    return new TrapezoidProfile.State(
        MathUtil.clamp(state.position, m_lowerLimitMeters, m_upperLimitMeters), state.velocity);
  }

  public void setSetpointTrapezoidState(TrapezoidProfile.State state) {
    elevatorMotors[0].set(
        TalonFXControlMode.Position,
        state.position / Constants.ELEVATOR.encoderCountsToMeters,
        DemandType.ArbitraryFeedForward,
        (m_feedForward.calculate(state.velocity) / 12.0));
  }

  public void resetState() {
    m_setpoint = new TrapezoidProfile.State(getHeightMeters(), getVelocityMeters());
  }

  // Elevator's height position
  public double getHeightMeters() {
    return elevatorMotors[0].getSelectedSensorPosition() * Constants.ELEVATOR.encoderCountsToMeters;
  }

  // Returns the elevator's velocity in meters per second.
  public double getVelocityMeters() {
    return elevatorMotors[0].getSelectedSensorVelocity()
        * Constants.ELEVATOR.encoderCountsToMeters
        * 10;
  }

  public double getHeightEncoderCounts() {
    return elevatorMotors[0].getSelectedSensorPosition();
  }

  public double getMotorOutputVoltage() {
    return elevatorMotors[0].getMotorOutputVoltage();
  }

  public boolean getIsElevating() {
    return isElevatorElevatingElevatando;
  }

  public boolean setIsElevating(boolean state) {
    return isElevatorElevatingElevatando = state;
  }

  public boolean getLimitSwitch() {
    return !lowerLimitSwitch.get();
  }

  public void setSensorPosition(double meters) {
    elevatorMotors[0].setSelectedSensorPosition(meters / Constants.ELEVATOR.encoderCountsToMeters);
  }

  public ELEVATOR.SETPOINT getSetpointState() {
    return m_desiredHeightState;
  }

  public boolean getElevatingState() {
    return !(Math.abs(getPercentOutput()) < 0.05);
  }

  public void setSetpointState(ELEVATOR.SETPOINT heightEnum) {
    m_desiredHeightState = heightEnum;
  }

  public void setDesiredPositionMeters(double meters) {
    m_desiredPositionInputMeters = meters;
  }

  public void setReduceCanUtilization(CAN_UTIL_LIMIT limitCan) {
    m_limitCanUtil = limitCan;
  }

  public double getDesiredPositionMeters() {
    return m_desiredPositionInputMeters;
  }

  public double getCommandedPositionMeters() {
    return m_commandedPositionMeters;
  }

  public void setLowerLimitMeters(double meters) {
    m_lowerLimitMeters = meters;
  }

  public double getLowerLimitMeters() {
    return m_lowerLimitMeters;
  }

  public void setUpperLimitMeters(double meters) {
    m_upperLimitMeters = meters;
  }

  public double getUpperLimitMeters() {
    return m_upperLimitMeters;
  }

  public void setJoystickY(double m_joystickY) {
    m_joystickInput = m_joystickY;
  }

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }

  public boolean isUserControlled() {
    return m_joystickInput != 0 && m_userSetpoint == false;
  }

  /*
   * Closed loop (default): Uses motion magic and a set setpoint to determine motor output
   * Open loop: Changes motor output directly
   */
  public void setControlMode(boolean isClosedLoop) {
    this.isClosedLoop = isClosedLoop;
  }

  public boolean getClosedLoopState() {
    return isClosedLoop;
  }

  public void setControlState(ELEVATOR.STATE state) {
    m_controlState = state;
  }

  public ELEVATOR.STATE getControlState() {
    return m_controlState;
  }

  public Translation2d getField2dTranslation() {
    return new Translation2d(
        -getHeightMeters() * Math.cos(Constants.ELEVATOR.mountAngleRadians.getRadians())
            + centerOffset,
        0);
  }

  private void initShuffleboard(CAN_UTIL_LIMIT limitCan) {
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Elevator Sim", mech2d);
    }
    SmartDashboard.putData("Elevator Subsystem", this);

    NetworkTable elevatorNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Elevator");

    kHeightPub = elevatorNtTab.getDoubleTopic("Height Meters").publish();
    kHeightInchesPub = elevatorNtTab.getDoubleTopic("Height Inches").publish();
    kDesiredHeightPub = elevatorNtTab.getDoubleTopic("Desired Height").publish();
    kEncoderCountsPub = elevatorNtTab.getDoubleTopic("Encoder Counts").publish();
    kDesiredStatePub = elevatorNtTab.getStringTopic("Desired State").publish();
    kPercentOutputPub = elevatorNtTab.getDoubleTopic("Percent Output").publish();
    kClosedLoopModePub = elevatorNtTab.getStringTopic("Closed-Loop Mode").publish();
    currentCommandStatePub = elevatorNtTab.getStringTopic("Current Command State").publish();
    lowerLimitSwitchPub = elevatorNtTab.getBooleanTopic("Lower Limit Switch").publish();

    elevatorNtTab.getDoubleTopic("kP").publish().set(Constants.ELEVATOR.kP);
    elevatorNtTab.getDoubleTopic("kI").publish().set(Constants.ELEVATOR.kI);
    elevatorNtTab.getDoubleTopic("kD").publish().set(Constants.ELEVATOR.kD);

    elevatorNtTab.getDoubleTopic("Max Vel").publish().set(Constants.ELEVATOR.kMaxVel);
    elevatorNtTab.getDoubleTopic("Max Accel").publish().set(Constants.ELEVATOR.kMaxAccel);
    elevatorNtTab.getDoubleTopic("kS").publish().set(Constants.ELEVATOR.kG);
    elevatorNtTab.getDoubleTopic("kV").publish().set(Constants.ELEVATOR.kV);
    elevatorNtTab.getDoubleTopic("kA").publish().set(Constants.ELEVATOR.kA);

    elevatorNtTab.getDoubleTopic("setpoint").publish().set(0);

    // Initialize Test Values
    // kPSub = elevatorNtTab.getDoubleTopic("kP").subscribe(kP);
    // kISub = elevatorNtTab.getDoubleTopic("kI").subscribe(kI);
    // kDSub = elevatorNtTab.getDoubleTopic("kD").subscribe(kD);

    // kMaxVelSub = elevatorNtTab.getDoubleTopic("Max Vel").subscribe(maxVel);
    // kMaxAccelSub = elevatorNtTab.getDoubleTopic("Max Accel").subscribe(maxAccel);
    // kSSub = elevatorNtTab.getDoubleTopic("kS").subscribe(kG);
    // kVSub = elevatorNtTab.getDoubleTopic("kV").subscribe(kV);
    // kASub = elevatorNtTab.getDoubleTopic("kA").subscribe(kA);

    kSetpointSub = elevatorNtTab.getDoubleTopic("setpoint").subscribe(0);
  }

  public void updateShuffleboard(CAN_UTIL_LIMIT limitCan) {
    SmartDashboard.putBoolean("Elevator Closed Loop", getClosedLoopState());
    SmartDashboard.putNumber("Elevator Height Inches", Units.metersToInches(getHeightMeters()));
    kClosedLoopModePub.set(isClosedLoop ? "Closed" : "Open");
    kHeightInchesPub.set(Units.metersToInches(getHeightMeters()));
    kDesiredHeightPub.set(getDesiredPositionMeters());

    switch (limitCan) {
      case NORMAL:
        // Put not required stuff here
        kEncoderCountsPub.set(getHeightEncoderCounts());
        kHeightPub.set(getHeightMeters());
        kDesiredStatePub.set(m_desiredHeightState.name());
        kPercentOutputPub.set(getPercentOutput());
        lowerLimitSwitchPub.set(getLimitSwitch());
        currentCommandStatePub.set(getControlState().toString());
        break;
      default:
      case LIMITED:
        lowerLimitSwitchPub.set(getLimitSwitch());
        kPercentOutputPub.set(getPercentOutput());
        break;
    }
  }

  public void updateLog() {
    outputCurrentEntry.append(elevatorMotors[0].getStatorCurrent());
    setpointMetersEntry.append(m_desiredPositionInputMeters);
    positionMetersEntry.append(heightMeters);
  }

  public void updateForwardOutput() {
    if (Units.metersToInches(getHeightMeters()) > 40.0) newForwardOutput = 0.2;
    else newForwardOutput = Constants.ELEVATOR.kMaxForwardOutput;

    if (currentForwardOutput != newForwardOutput) {
      elevatorMotors[0].configPeakOutputForward(newForwardOutput);
      currentForwardOutput = newForwardOutput;
    }
  }

  // Update elevator height using encoders and bottom limit switch
  public void updateHeightMeters() {
    /* Uses limit switch to act as a baseline
     * to reset the sensor position and height to improve accuracy
     */
    if (getLimitSwitch() && !lowerLimitSwitchTriggered) {
      setSensorPosition(0.0);
      lowerLimitSwitchTriggered = true;
    } else if (!getLimitSwitch() && lowerLimitSwitchTriggered) {
      lowerLimitSwitchTriggered = false;
    }
    heightMeters = getHeightMeters();
  }

  @Override
  public void simulationPeriodic() {
    elevatorSim.setInput(getPercentOutput() * 12);

    // Next, we update it. The standard loop time is 20ms.
    double currentTime = m_timer.get();
    elevatorSim.update(currentTime - m_lastSimTimestamp);
    m_lastSimTimestamp = currentTime;

    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (simEncoderSign
                    * elevatorSim.getPositionMeters()
                    / Constants.ELEVATOR.encoderCountsToMeters));

    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (simEncoderSign
                    * elevatorSim.getVelocityMetersPerSecond()
                    / Constants.ELEVATOR.encoderCountsToMeters
                    * 10));

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // This is why the mech2d is not proportional. We're using Units.metersToInches instead of
    // directly setting the length to meters
    // TODO: Make the mech2d proportional
    elevatorLigament2d.setLength(Units.metersToInches(elevatorSim.getPositionMeters()));
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateLog();
    updateShuffleboard(
        m_limitCanUtil); // Yes, this needs to be called in the periodic. The simulation does not
    // work without this
    updateHeightMeters();
    updateForwardOutput();

    if (isClosedLoop) {
      switch (m_controlState) {
        case CLOSED_LOOP_MANUAL:
          m_desiredPositionOutputMeters =
              MathUtil.clamp(
                  m_joystickInput * Constants.ELEVATOR.kSetpointMultiplier + getHeightMeters(),
                  ELEVATOR.THRESHOLD.ABSOLUTE_MIN.get(),
                  ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get());
          break;
        case OPEN_LOOP_MANUAL:
          double percentOutput = m_joystickInput * Constants.ELEVATOR.kPercentOutputMultiplier;
          if (getHeightMeters() > (getUpperLimitMeters() - 0.0254)) {
            percentOutput = Math.min(percentOutput, 0);
          }
          if (getHeightMeters() < (getLowerLimitMeters() + 0.005)) {
            percentOutput = Math.max(percentOutput, 0);
          }
          setPercentOutput(percentOutput);
          break;
        case USER_SETPOINT:
          m_desiredPositionOutputMeters =
              m_desiredPositionInputMeters
                  + m_joystickInput * Constants.ELEVATOR.kSetpointMultiplier;
          break;
        case TEST_SETPOINT:
          m_desiredPositionOutputMeters = Units.inchesToMeters(kSetpointSub.get(0));
          break;
        default:
        case AUTO_SETPOINT:
          m_desiredPositionOutputMeters = m_desiredPositionInputMeters;
          break;
      }
      if (DriverStation.isEnabled() && m_controlState != ELEVATOR.STATE.OPEN_LOOP_MANUAL) {
        if (m_desiredPositionInputMeters - getHeightMeters() > 0)
          m_currentConstraints = m_fastConstraints;
        else if (getHeightMeters() < Units.inchesToMeters(3.0)) {
          m_currentConstraints = m_stopSlippingConstraints;
        } else m_currentConstraints = m_slowConstraints;

        m_goal = new TrapezoidProfile.State(m_desiredPositionOutputMeters, 0);
        TrapezoidProfile profile = new TrapezoidProfile(m_currentConstraints, m_goal, m_setpoint);
        double currentTime = m_timer.get();
        m_setpoint = profile.calculate(currentTime - m_lastTimestamp);
        m_lastTimestamp = currentTime;

        var commandedSetpoint = limitDesiredSetpointMeters(m_setpoint);
        m_commandedPositionMeters = commandedSetpoint.position;
        kDesiredHeightPub.set(Units.metersToInches(commandedSetpoint.position));
        setSetpointTrapezoidState(commandedSetpoint);
      }
      // Fully open loop control
    } else {
      double percentOutput = m_joystickInput * Constants.ELEVATOR.kPercentOutputMultiplier;
      if (getHeightMeters() > (getUpperLimitMeters() - 0.0254)) {
        percentOutput = Math.min(percentOutput, 0);
      }
      if (getHeightMeters() < (getLowerLimitMeters() + 0.0015)) {
        percentOutput = Math.max(percentOutput, 0);
      }
      setPercentOutput(percentOutput);
    }
  }

  @Override
  public void close() throws Exception {
    lowerLimitSwitch.close();
  }
}
