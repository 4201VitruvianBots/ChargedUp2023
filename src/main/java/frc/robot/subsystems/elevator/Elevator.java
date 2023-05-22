// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ELEVATOR.centerOffset;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.ELEVATOR.THRESHOLD;
import frc.robot.Constants.STATE_HANDLER;

public class Elevator extends SubsystemBase implements AutoCloseable {
  // private boolean m_elevatorInitialized;

  // Initializing limit switch at bottom of elevator
  //  private final DigitalInput lowerLimitSwitch = new DigitalInput(DIO.elevatorLowerLimitSwitch);
  //  private boolean lowerLimitSwitchTriggered = false;

  public final ElevatorIO m_io;
  private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();

  private double m_desiredPositionMeters; // The height in meters our robot is trying to reach

  private CONTROL_MODE m_controlMode = CONTROL_MODE.CLOSED_LOOP;
  private boolean m_testMode = false;

  private final boolean m_limitCanUtil = STATE_HANDLER.limitCanUtilization;

  // Positional limits set by the state handler
  private double m_lowerLimitMeters = THRESHOLD.ABSOLUTE_MIN.get();
  private double m_upperLimitMeters = THRESHOLD.ABSOLUTE_MAX.get();

  // Controlled by open loop
  private double m_joystickInput;
  private boolean m_limitJoystickInput;
  private boolean m_userSetpoint;

  // Trapezoid profile setup
  private TrapezoidProfile m_currentProfile;
  private TrapezoidProfile.Constraints m_currentConstraints = ELEVATOR.m_Constraints;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private final SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(ELEVATOR.kG, ELEVATOR.kV, ELEVATOR.kA);
  private SimpleMotorFeedforward m_currentFeedForward = m_feedForward;
  // This timer is used to calculate the time since the previous periodic run to determine our new
  // setpoint
  private final Timer m_timer = new Timer();
  private boolean m_unitTestBoolean = false; // DO NOT MAKE FINAL. WILL BREAK UNIT TESTS
  private double m_lastTimestamp = 0;
  private double m_currentTimestamp = 0;

  // Shuffleboard setup
  private DoublePublisher kHeightPub,
      kEncoderCountsPub,
      kDesiredHeightPub,
      kHeightInchesPub,
      kPercentOutputPub,
      kCurrentAccelPub,
      kCurrentVelPub;
  private StringPublisher kClosedLoopModePub, currentCommandStatePub;
  private BooleanPublisher lowerLimitSwitchPub;

  // Logging setup
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry outputCurrentEntry = new DoubleLogEntry(log, "/elevator/current");
  private final DoubleLogEntry setpointMetersEntry = new DoubleLogEntry(log, "/elevator/setpoint");
  private final DoubleLogEntry positionMetersEntry = new DoubleLogEntry(log, "/elevator/position");

  /* Constructs a new Elevator. Mostly motor setup */
  public Elevator(ElevatorIO io) {
    m_io = io;

    initShuffleboard();
    m_timer.reset();
    m_timer.start();
  }

  // Elevator's motor output as a percentage
  public double getPercentOutput() {
    return m_inputs.percentOutput;
  }

  // Setting the raw output of the motors
  public void setPercentOutput(double output, boolean enforceLimits) {
    // If we have activated our limit switch, do not move the elevator backward
    if (getLimitSwitch() && output < 0) output = Math.max(output, 0);

    // Enforce limits if requested
    if (enforceLimits) {
      if (getHeightMeters() > (getUpperLimitMeters() - Units.inchesToMeters(1)))
        output = Math.min(output, 0);
      if (getHeightMeters() < (getLowerLimitMeters() + 0.005)) output = Math.max(output, 0);
    }

    m_io.setPercentOutput(output);
  }

  public void setPercentOutput(double output) {
    setPercentOutput(output, false);
  }

  private double calculateFeedforward(TrapezoidProfile.State state) {
    return (m_feedForward.calculate(state.position, state.velocity) / 12.0);
  }

  // Sets the setpoint to our current height, effectively keeping the elevator in place.
  public void resetTrapezoidState() {
    m_setpoint = new TrapezoidProfile.State(getHeightMeters(), getVelocityMetersPerSecond());
  }

  // Elevator's height position
  public double getHeightMeters() {
    return getHeightEncoderCounts() * ELEVATOR.encoderCountsToMeters;
  }

  // Returns the elevator's velocity in meters per second.
  public double getVelocityMetersPerSecond() {
    return m_inputs.velocityMetersPerSec;
  }

  // Returns the raw sensor position with no conversions
  public double getHeightEncoderCounts() {
    return m_inputs.heightEncoderCounts;
  }

  public void setPIDvalues(double f, double p, double i, double d, double iZone) {
    m_io.setPIDvalues(f, p, i, d, iZone);
  }

  public void setNeutralMode(NeutralMode mode) {
    m_io.setNeutralMode(mode);
  }

  public void setSensorPosition(double meters) {
    m_io.setSensorPosition(meters);
  }

  // Returns true if elevator is within half of an inch of its set position
  // This means that the elevator is only trying to hold its current setpoint, not move towards a
  // new one
  public boolean atSetpoint() {
    return Math.abs(getHeightMeters() - getDesiredPositionMeters()) < Units.metersToInches(0.5);
  }

  // Returns true if limit switch is activated
  public boolean getLimitSwitch() {
    //    return !lowerLimitSwitch.get();
    return false;
  }

  // This is so incredibly scuffed but it'll have to do
  public NeutralMode getNeutralMode() {
    switch (m_inputs.neutralMode) {
      case "Brake":
        return NeutralMode.Brake;
      case "Coast":
        return NeutralMode.Coast;
      default:
      case "EEPROM":
        return NeutralMode.EEPROMSetting;
    }
  }

  public void setDesiredPositionMeters(double meters) {
    m_desiredPositionMeters = meters;
  }

  public double getDesiredPositionMeters() {
    return m_desiredPositionMeters;
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

  public void setTestMode(boolean mode) {
    m_testMode = mode;
  }

  public void setSimpleMotorFeedForward(double g, double v, double a) {
    m_currentFeedForward = new SimpleMotorFeedforward(g, v, a);
  }

  public void setTrapezoidalConstraints(double maxVel, double maxAccel) {
    m_currentConstraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
  }

  public void setJoystickLimit(boolean limit) {
    m_limitJoystickInput = limit;
  }

  // True when moving the joystick up and down to control the elevator instead of buttons, in either
  // open or closed loop
  public boolean isUserControlled() {
    return m_joystickInput != 0 && !m_userSetpoint;
  }

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }

  // Sets the control state of the elevator
  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  // Returns the current control state enum
  public CONTROL_MODE getClosedLoopControlMode() {
    return m_controlMode;
  }

  public boolean isClosedLoopControl() {
    return getClosedLoopControlMode() == CONTROL_MODE.CLOSED_LOOP;
  }

  // Returns a translation of the elevator's position in relation to the robot's position.
  public Translation2d getField2dTranslation() {
    return new Translation2d(
        -getHeightMeters() * Math.cos(ELEVATOR.mountAngleRadians.getRadians()) - centerOffset, 0);
  }

  // Returns the ligament of the elevator to update in StateHandler
  public MechanismLigament2d getLigament() {
    return m_io.getLigament();
  }

  public void setLigament(MechanismLigament2d ligament) {
    m_io.setLigament(ligament);
  }

  // Initializes shuffleboard values. Does not update them
  private void initShuffleboard() {
    SmartDashboard.putData("Elevator Subsystem", this);

    NetworkTable elevatorNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Elevator");

    kHeightPub = elevatorNtTab.getDoubleTopic("Height Meters").publish();
    kHeightInchesPub = elevatorNtTab.getDoubleTopic("Height Inches").publish();
    kDesiredHeightPub = elevatorNtTab.getDoubleTopic("Desired Height Inches").publish();
    kEncoderCountsPub = elevatorNtTab.getDoubleTopic("Encoder Counts").publish();
    kPercentOutputPub = elevatorNtTab.getDoubleTopic("Percent Output").publish();
    kClosedLoopModePub = elevatorNtTab.getStringTopic("Closed-Loop Mode").publish();
    currentCommandStatePub = elevatorNtTab.getStringTopic("Current Command State").publish();
    lowerLimitSwitchPub = elevatorNtTab.getBooleanTopic("Lower Limit Switch").publish();
    kCurrentAccelPub = elevatorNtTab.getDoubleTopic("Current Acceleration").publish();
    kCurrentVelPub = elevatorNtTab.getDoubleTopic("Current Velocity").publish();

    try {
      elevatorNtTab.getDoubleTopic("setpoint").publish().set(0);
    } catch (Exception m_ignored) {

    }
  }

  public void updateShuffleboard() {
    SmartDashboard.putBoolean("Elevator Closed Loop", isClosedLoopControl());
    SmartDashboard.putNumber("Elevator Height Inches", Units.metersToInches(getHeightMeters()));

    kClosedLoopModePub.set(getClosedLoopControlMode().toString());
    kHeightInchesPub.set(Units.metersToInches(getHeightMeters()));
    kDesiredHeightPub.set(Units.metersToInches(getDesiredPositionMeters()));
    lowerLimitSwitchPub.set(getLimitSwitch());
    kCurrentAccelPub.set(Units.metersToInches(m_currentConstraints.maxAcceleration));
    kCurrentVelPub.set(Units.metersToInches(m_currentConstraints.maxVelocity));

    if (!m_limitCanUtil) {
      // Put not required stuff here
      kEncoderCountsPub.set(getHeightEncoderCounts());
      kHeightPub.set(getHeightMeters());
      kPercentOutputPub.set(getPercentOutput());
      currentCommandStatePub.set(getClosedLoopControlMode().toString());
    }
  }

  public void updateLog() {
    outputCurrentEntry.append(m_inputs.outputCurrent);
    setpointMetersEntry.append(m_desiredPositionMeters);
    positionMetersEntry.append(getHeightMeters());
  }

  // Update elevator height using encoders and bottom limit switch
  private void updateHeightMeters() {
    /* Uses limit switch to act as a baseline
     * to reset the sensor position and height to improve accuracy
     */
    //    if (getLimitSwitch() && !lowerLimitSwitchTriggered) {
    //      setSensorPosition(0.0);
    //      lowerLimitSwitchTriggered = true;
    //    } else if (!getLimitSwitch() && lowerLimitSwitchTriggered) {
    //      lowerLimitSwitchTriggered = false;
    //    }
  }

  public void teleopInit() {
    setDesiredPositionMeters(getHeightMeters());
    resetTrapezoidState();
  }

  @Override
  public void simulationPeriodic() {
    m_io.simulationPeriodic();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // initElevatorMotorFollower();
    updateLog();
    updateShuffleboard(); // Yes, this needs to be called in the periodic. The simulation does not
    // work without this
    if (!m_testMode) {
      updateHeightMeters();
    }

    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Elevator", m_inputs);

    switch (m_controlMode) {
        // Called when setting to open loop
      case OPEN_LOOP:
        double percentOutput = m_joystickInput * ELEVATOR.kPercentOutputMultiplier;

        // Limit the percent output of the elevator joystick when the stick is pressed down to make
        // small adjustments
        if (m_limitJoystickInput)
          percentOutput = m_joystickInput * ELEVATOR.kLimitedPercentOutputMultiplier;

        // Sets final percent output
        // True means it will enforce limits. In this way it is not truly open loop, but it'll
        // prevent the robot from breaking
        setPercentOutput(percentOutput, true);
        break;
      default:
      case CLOSED_LOOP:
        // Updates our trapezoid profile state based on the time since our last periodic and our
        // recorded change in height
        m_goal = new TrapezoidProfile.State(m_desiredPositionMeters, 0);
        m_currentProfile = new TrapezoidProfile(m_currentConstraints, m_goal, m_setpoint);
        m_currentTimestamp = m_timer.get();
        m_setpoint = m_currentProfile.calculate(m_currentTimestamp - m_lastTimestamp);
        m_lastTimestamp = m_currentTimestamp;

        m_io.setSetpointTrapezoidState(m_setpoint);
        break;
    }
  }

  @Override
  // Safely closes the subsystem
  public void close() throws Exception {
    m_io.close();
  }
}
