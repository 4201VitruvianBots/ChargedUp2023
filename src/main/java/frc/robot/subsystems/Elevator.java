// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ELEVATOR.centerOffset;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
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
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_UTIL_LIMIT;
import frc.robot.Constants.ELEVATOR;

public class Elevator extends SubsystemBase implements AutoCloseable {

  // Initializing both motors
  private final TalonFX[] elevatorMotors = {
    new TalonFX(Constants.CAN.elevatorMotorLeft), new TalonFX(Constants.CAN.elevatorMotorRight)
  };

  // Initializing limit switch at bottom of elevator
  private final DigitalInput lowerLimitSwitch =
      new DigitalInput(Constants.DIO.elevatorLowerLimitSwitch);
  private boolean lowerLimitSwitchTriggered = false;

  private final double maxHeightMeters = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get();

  private double m_desiredPositionMeters; // The height in meters our robot is trying to reach
  private double m_commandedPositionMeters; // The height in meters our robot is trying to reach
  private ELEVATOR.SETPOINT m_desiredHeightState = ELEVATOR.SETPOINT.STOWED;
  // TODO: Unify this to one enum
  private ELEVATOR.STATE m_closedLoopControl = ELEVATOR.STATE.CLOSED_LOOP;
  // TODO: Move to StateHandler/make global
  private CAN_UTIL_LIMIT m_limitCanUtil = CAN_UTIL_LIMIT.NORMAL;

  // TODO: Review if this limit is necessary if we are already using trapezoidal profiles
  // This is used in limiting the elevator's speed once we reach the top of the elevator
  private double currentForwardOutput = 0;
  private double newForwardOutput = 0;

  // Positional limits set by the state handler
  private double m_lowerLimitMeters = ELEVATOR.THRESHOLD.ABSOLUTE_MIN.get();
  private double m_upperLimitMeters = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get();

  // Controlled by open loop
  private double m_joystickInput;
  private boolean m_userSetpoint;

  // Trapezoid profile setup
  private TrapezoidProfile.Constraints m_currentConstraints = Constants.ELEVATOR.m_slowConstraints;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private SimpleMotorFeedforward m_feedForward =
      new SimpleMotorFeedforward(
          Constants.ELEVATOR.kG, Constants.ELEVATOR.kV, Constants.ELEVATOR.kA);
  // This timer is used to calculate the time since the previous periodic run to determine our new
  // setpoint
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
  public Mechanism2d mech2d = new Mechanism2d(maxHeightMeters * 1.5, maxHeightMeters * 1.5);
  public MechanismRoot2d root2d =
      mech2d.getRoot("Elevator", maxHeightMeters * 0.5, maxHeightMeters * 0.5);
  public MechanismLigament2d elevatorLigament2d =
      root2d.append(
          new MechanismLigament2d(
              "Elevator",
              getHeightMeters() + Constants.ELEVATOR.carriageDistance,
              Constants.ELEVATOR.angleDegrees));
  public MechanismLigament2d robotBase2d =
      root2d.append(new MechanismLigament2d("Robot Base", Constants.SWERVEDRIVE.kTrackWidth, 0));

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

      // Setting hard limits as to how fast the elevator can move forward and backward
      motor.configPeakOutputForward(
          Constants.ELEVATOR.kMaxForwardOutput, Constants.ELEVATOR.kTimeoutMs);
      motor.configPeakOutputReverse(
          Constants.ELEVATOR.kMaxReverseOutput, Constants.ELEVATOR.kTimeoutMs);
      // TODO: Review after new elevator is integrated
      motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 0.1));
    }

    // Setting the right motor to output the same as the left motor
    elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());

    elevatorMotors[0].setInverted(Constants.ELEVATOR.mainMotorInversionType);
    elevatorMotors[1].setInverted(TalonFXInvertType.OpposeMaster);
    elevatorMotors[1].setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    elevatorMotors[1].setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    initShuffleboard();
    m_timer.reset();
    m_timer.start();
  }

  // Elevator's motor output as a percentage
  public double getPercentOutput() {
    return elevatorMotors[0].getMotorOutputPercent();
  }

  public void setPercentOutput(double output) {
    setPercentOutput(output, false);
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

    elevatorMotors[0].set(ControlMode.PercentOutput, output);
  }

  // TODO: Remove
  // Clamp the desired setpoint to within our lower and upper limits
  private TrapezoidProfile.State limitDesiredSetpointMeters(TrapezoidProfile.State state) {
    return new TrapezoidProfile.State(
        MathUtil.clamp(state.position, m_lowerLimitMeters, m_upperLimitMeters), state.velocity);
  }

  // Sets the calculated trapezoid state of the motors
  public void setSetpointTrapezoidState(TrapezoidProfile.State state) {
    elevatorMotors[0].set(
        TalonFXControlMode.Position,
        state.position / Constants.ELEVATOR.encoderCountsToMeters,
        DemandType.ArbitraryFeedForward,
        (m_feedForward.calculate(state.velocity) / 12.0));
  }

  // Sets the setpoint to our current height, effectively keeping the elevator in place.
  public void haltPosition() {
    m_setpoint = new TrapezoidProfile.State(getHeightMeters(), getVelocityMetersPerSecond());
  }

  // Elevator's height position
  public double getHeightMeters() {
    return getHeightEncoderCounts() * Constants.ELEVATOR.encoderCountsToMeters;
  }

  // Returns the elevator's velocity in meters per second.
  public double getVelocityMetersPerSecond() {
    return elevatorMotors[0].getSelectedSensorVelocity()
        * Constants.ELEVATOR.encoderCountsToMeters
        * 10;
  }

  // Returns the raw sensor position with no conversions
  public double getHeightEncoderCounts() {
    return elevatorMotors[0].getSelectedSensorPosition();
  }

  // Returns true if elevator is within half of an inch of its set position
  // This means that the elevator is only trying to hold its current setpoint, not move towards a
  // new one
  public boolean atSetpoint() {
    return Math.abs(getHeightMeters() - getCommandedPositionMeters()) < Units.metersToInches(0.5);
  }

  // Returns true if limit switch is activated
  public boolean getLimitSwitch() {
    return !lowerLimitSwitch.get();
  }

  // Sets the perceived position of the motors
  // Usually used to zero the motors if the robot is started in a non-stowed position
  public void setSensorPosition(double meters) {
    elevatorMotors[0].setSelectedSensorPosition(meters / Constants.ELEVATOR.encoderCountsToMeters);
  }

  public void setDesiredPositionMeters(double meters) {
    m_desiredPositionMeters = meters;
  }

  public void setReduceCanUtilization(CAN_UTIL_LIMIT limitCan) {
    m_limitCanUtil = limitCan;
  }

  public double getDesiredPositionMeters() {
    return m_desiredPositionMeters;
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

  // True when moving the joystick up and down to control the elevator instead of buttons, in either
  // open or closed loop
  public boolean isUserControlled() {
    return m_joystickInput != 0 && m_userSetpoint == false;
  }

  // Sets the control state of the elevator
  public void setClosedLoopControlState(ELEVATOR.STATE state) {
    m_closedLoopControl = state;
  }

  // Returns the current control state enum
  public ELEVATOR.STATE getClosedLoopControlState() {
    return m_closedLoopControl;
  }

  public boolean isClosedLoopControl() {
    return getClosedLoopControlState() == ELEVATOR.STATE.CLOSED_LOOP;
  }

  // Returns a translation of the elevator's position in relation to the robot's position.
  public Translation2d getField2dTranslation() {
    return new Translation2d(
        -getHeightMeters() * Math.cos(Constants.ELEVATOR.mountAngleRadians.getRadians())
            + centerOffset,
        0);
  }

  // Returns the ligament of the elevator so the wrist ligament can be attached to it
  public MechanismLigament2d getLigament2d() {
    return elevatorLigament2d;
  }

  // Initializes shuffleboard values. Does not update them
  private void initShuffleboard() {
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Elevator Sim", mech2d);
    }
    SmartDashboard.putData("Elevator Subsystem", this);

    NetworkTable elevatorNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Elevator");

    // Change the color of the robot base mech2d
    robotBase2d.setColor(new Color8Bit(173, 216, 230)); // Light blue

    kHeightPub = elevatorNtTab.getDoubleTopic("Height Meters").publish();
    kHeightInchesPub = elevatorNtTab.getDoubleTopic("Height Inches").publish();
    kDesiredHeightPub = elevatorNtTab.getDoubleTopic("Desired Height").publish();
    kEncoderCountsPub = elevatorNtTab.getDoubleTopic("Encoder Counts").publish();
    kDesiredStatePub = elevatorNtTab.getStringTopic("Desired State").publish();
    kPercentOutputPub = elevatorNtTab.getDoubleTopic("Percent Output").publish();
    kClosedLoopModePub = elevatorNtTab.getStringTopic("Closed-Loop Mode").publish();
    currentCommandStatePub = elevatorNtTab.getStringTopic("Current Command State").publish();
    lowerLimitSwitchPub = elevatorNtTab.getBooleanTopic("Lower Limit Switch").publish();

    // TODO: Rewrite test interface
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

  public void updateShuffleboard() {
    SmartDashboard.putBoolean(
        "Elevator Closed Loop", getClosedLoopControlState() == ELEVATOR.STATE.CLOSED_LOOP);
    SmartDashboard.putNumber("Elevator Height Inches", Units.metersToInches(getHeightMeters()));
    kClosedLoopModePub.set(
        getClosedLoopControlState() == ELEVATOR.STATE.CLOSED_LOOP ? "Closed" : "Open");
    kHeightInchesPub.set(Units.metersToInches(getHeightMeters()));
    kDesiredHeightPub.set(getDesiredPositionMeters());
    lowerLimitSwitchPub.set(getLimitSwitch());
    kPercentOutputPub.set(getPercentOutput());

    if (m_limitCanUtil == CAN_UTIL_LIMIT.NORMAL) {
      // Put not required stuff here
      kEncoderCountsPub.set(getHeightEncoderCounts());
      kHeightPub.set(getHeightMeters());
      kDesiredStatePub.set(m_desiredHeightState.name());
      kPercentOutputPub.set(getPercentOutput());
      lowerLimitSwitchPub.set(getLimitSwitch());
      currentCommandStatePub.set(getClosedLoopControlState().toString());
    }
  }

  public void updateLog() {
    outputCurrentEntry.append(elevatorMotors[0].getStatorCurrent());
    setpointMetersEntry.append(m_desiredPositionMeters);
    positionMetersEntry.append(getHeightMeters());
  }

  // Will severely limit the forward output of the motors when the elevator is fully extended to
  // prevent breakage
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
  }

  @Override
  public void simulationPeriodic() {
    elevatorSim.setInput(getPercentOutput() * 12);

    // Next, we update it. The standard loop time is 20ms.
    double currentTime = m_timer.get();
    elevatorSim.update(currentTime - m_lastSimTimestamp);
    m_lastSimTimestamp = currentTime;

    // Internally sets the position of the motors in encoder counts based on our current height in
    // meters
    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (Constants.ELEVATOR.simEncoderSign
                    * elevatorSim.getPositionMeters()
                    / Constants.ELEVATOR.encoderCountsToMeters));

    // Internally sets the velocity of the motors in encoder counts per 100 ms based on our velocity
    // in meters per second (1000 ms)
    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (Constants.ELEVATOR.simEncoderSign
                    * elevatorSim.getVelocityMetersPerSecond()
                    / Constants.ELEVATOR.encoderCountsToMeters
                    * 10));

    // Sets the simulated voltage of the roboRio based on our current draw from the elevator
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // This is why the mech2d is not proportional. We're using Units.metersToInches instead of
    // directly setting the length to meters
    elevatorLigament2d.setLength(
        elevatorSim.getPositionMeters() + Constants.ELEVATOR.carriageDistance);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateLog();
    updateShuffleboard(); // Yes, this needs to be called in the periodic. The simulation does not
    // work without this
    updateHeightMeters();
    updateForwardOutput();

    switch (m_closedLoopControl) {
        // Called when setting to open loop
      case OPEN_LOOP_MANUAL:
        double percentOutput = m_joystickInput * Constants.ELEVATOR.kPercentOutputMultiplier;
        // Sets final percent output
        // True means it will enforce limits. In this way it is not truly open loop, but it'll
        // prevent the robot from breaking
        setPercentOutput(percentOutput, true);
        break;
      default:
      case CLOSED_LOOP:
        // Updates the constraints of the elevator
        if (m_desiredPositionMeters - getHeightMeters() > 0) {
          m_currentConstraints = Constants.ELEVATOR.m_fastConstraints;
        } else if (getHeightMeters() < Units.inchesToMeters(3.0)) {
          m_currentConstraints = Constants.ELEVATOR.m_stopSlippingConstraints;
        } else {
          m_currentConstraints = Constants.ELEVATOR.m_slowConstraints;
        }

        // Updates our trapezoid profile state based on the time since our last periodic and our
        // recorded change in height
        m_goal = new TrapezoidProfile.State(m_desiredPositionMeters, 0);
        TrapezoidProfile profile = new TrapezoidProfile(m_currentConstraints, m_goal, m_setpoint);
        double currentTime = m_timer.get();
        m_setpoint = profile.calculate(currentTime - m_lastTimestamp);
        m_lastTimestamp = currentTime;

        TrapezoidProfile.State commandedSetpoint = limitDesiredSetpointMeters(m_setpoint);
        m_commandedPositionMeters = commandedSetpoint.position;
        kDesiredHeightPub.set(Units.metersToInches(commandedSetpoint.position));
        setSetpointTrapezoidState(commandedSetpoint);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    elevatorSim.setInput(
        MathUtil.clamp(getPercentOutput() * RobotController.getBatteryVoltage(), -12, 12));

    // Next, we update it. The standard loop time is 20ms.
    double currentTime = m_timer.get();
    elevatorSim.update(currentTime - m_lastSimTimestamp);
    m_lastSimTimestamp = currentTime;

    // Internally sets the position of the motors in encoder counts based on our current height in
    // meters
    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (Constants.ELEVATOR.simEncoderSign
                    * elevatorSim.getPositionMeters()
                    / Constants.ELEVATOR.encoderCountsToMeters));

    // Internally sets the velocity of the motors in encoder counts per 100 ms based on our velocity
    // in meters per second (1000 ms)
    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (Constants.ELEVATOR.simEncoderSign
                    * elevatorSim.getVelocityMetersPerSecond()
                    / Constants.ELEVATOR.encoderCountsToMeters
                    * 10));

    // Sets the simulated voltage of the roboRio based on our current draw from the elevator
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // This is why the mech2d is not proportional. We're using Units.metersToInches instead of
    // directly setting the length to meters
    elevatorLigament2d.setLength(elevatorSim.getPositionMeters());
  }

  @Override
  // Safely closes the subsystem
  public void close() throws Exception {
    lowerLimitSwitch.close();
  }
}
