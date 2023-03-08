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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;

public class Elevator extends SubsystemBase {

  // Initializing both motors
  public final TalonFX[] elevatorMotors = {
    new TalonFX(Constants.CAN.elevatorMotorLeft), new TalonFX(Constants.CAN.elevatorMotorRight)
  };

  // Limit switch at bottom of elevator
  private final DigitalInput elevatorLowerSwitch =
      new DigitalInput(Constants.DIO.elevatorLowerSwitch);

  private double
      m_desiredPositionMeters; // The height in encoder units our robot is trying to reach
  private double
      m_commandedPositionMeters; // The height in encoder units our robot is trying to reach
  private ELEVATOR.SETPOINT desiredHeightState =
      ELEVATOR.SETPOINT.STOWED; // Think of this as our "next state" in our state machine.

  private double m_lowerLimitMeters = ELEVATOR.THRESHOLD.ABSOLUTE_MIN.get();
  private double m_upperLimitMeters = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get();

  private double joystickInput;

  private final double kP = 0.55;
  private final double kI = 0;
  private final double kD = 0;

  private double maxVel = ELEVATOR.kMaxVel;
  private double maxAccel = ELEVATOR.kMaxAccel;
  private double kS = ELEVATOR.kS;
  private double kV = ELEVATOR.kV;
  private double kA = ELEVATOR.kA;
  private TrapezoidProfile.Constraints m_trapezoidialConstraints =
      new TrapezoidProfile.Constraints(maxVel, maxAccel);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(kS, kV, kA);

  private static double elevatorHeight =
      0; // the amount of meters the motor has gone up from the initial stowed position

  private final double maxElevatorHeight = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get();

  // By default, this is set to true as we use motion magic to determine what speed we should be at
  // to get to our setpoint.
  // If the sensors are acting up, we set this value false to directly control the percent output
  // of the motors.
  private boolean isClosedLoop = true;
  public boolean isElevatorElevatingElevatando = false;
  private final int simEncoderSign =
      Constants.ELEVATOR.mainMotorInversionType == TalonFXInvertType.Clockwise ? -1 : 1;
  private ELEVATOR.STATE m_controlState = ELEVATOR.STATE.AUTO_SETPOINT;

  private final double maxPercentOutput = 0.75;
  private final double percentOutputMultiplier = 0.75;
  public final double setpointMultiplier = 0.50;

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

  private DoubleSubscriber kPSub,
      kISub,
      kDSub,
      kSetpointSub,
      kMaxVelSub,
      kMaxAccelSub,
      kSSub,
      kVSub,
      kASub;
  private DoublePublisher kHeightPub, kEncoderCountsPub, kDesiredHeightPub, kHeightInchesPub;
  private StringPublisher kDesiredStatePub,
      kPercentOutputPub,
      kClosedLoopModePub,
      currentCommandStatePub;

  // Mechanism2d visualization setup
  public Mechanism2d mech2d = new Mechanism2d(maxElevatorHeight * 50, maxElevatorHeight * 50);
  public MechanismRoot2d root2d = mech2d.getRoot("Elevator", maxElevatorHeight * 25, 0);
  public MechanismLigament2d elevatorLigament2d =
      root2d.append(new MechanismLigament2d("Elevator", elevatorHeight, 90));

  // Logging setup
  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry elevatorCurrentEntry = new DoubleLogEntry(log, "/elevator/elevatorCurrent");
  public DoubleLogEntry elevatorSetpointEntry =
      new DoubleLogEntry(log, "/elevator/elevatorSetpoint");
  public DoubleLogEntry elevatorPositionEntry =
      new DoubleLogEntry(log, "/elevator/elevatorPosition");

  /* Constructs a new Elevator. Mostly motor setup */
  public Elevator() {
    for (TalonFX motor : elevatorMotors) {
      motor.configFactoryDefault();
      motor.setNeutralMode(NeutralMode.Brake);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(0.0);

      // Config PID
      motor.selectProfileSlot(Constants.ELEVATOR.kSlotIdx, Constants.ELEVATOR.kPIDLoopIdx);
      motor.config_kP(Constants.ELEVATOR.kSlotIdx, kP, Constants.ELEVATOR.kTimeoutMs);
      motor.config_kI(Constants.ELEVATOR.kSlotIdx, kI, Constants.ELEVATOR.kTimeoutMs);
      motor.config_kD(Constants.ELEVATOR.kSlotIdx, kD, Constants.ELEVATOR.kTimeoutMs);

      motor.configPeakOutputForward(0.43, Constants.ELEVATOR.kTimeoutMs);
      motor.configPeakOutputReverse(-0.3, Constants.ELEVATOR.kTimeoutMs);
    }

    elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());

    elevatorMotors[0].setInverted(Constants.ELEVATOR.mainMotorInversionType);
    elevatorMotors[1].setInverted(TalonFXInvertType.OpposeMaster);

    initShuffleboard();
  }

  /*
   * Elevator's motor output as a percentage
   */
  public double getPercentOutput() {
    return elevatorMotors[0].getMotorOutputPercent();
  }

  public void setPercentOutput(double output) {
    // if (getElevatorLowerSwitch())
    //   MathUtil.clamp(output, 0, 1);
    elevatorMotors[0].set(ControlMode.PercentOutput, output);
  }

  public void setElevatorMotionMagicMeters(double setpoint) {
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
    m_setpoint = new TrapezoidProfile.State(getHeightMeters(), getVelocityMps());
  }

  /*
   * Elevator's height position
   */
  public double getHeightMeters() {
    return elevatorMotors[0].getSelectedSensorPosition() * Constants.ELEVATOR.encoderCountsToMeters;
  }

  public double getVelocityMps() {
    return elevatorMotors[0].getSelectedSensorVelocity()
        * Constants.ELEVATOR.encoderCountsToMeters
        * 10;
  }

  public double getElevatorEncoderCounts() {
    return elevatorMotors[0].getSelectedSensorPosition();
  }

  public double getElevatorMotorVoltage() {
    return elevatorMotors[0].getMotorOutputVoltage();
  }

  public boolean getElevatorRunning() {
    return isElevatorElevatingElevatando;
  }

  public boolean setElevatorRunning(boolean state) {
    return isElevatorElevatingElevatando = state;
  }

  // public boolean getElevatorLowerSwitch() {
  //   // return !elevatorLowerSwitch.get();
  // }

  public void setElevatorSensorPosition(double meters) {
    elevatorMotors[0].setSelectedSensorPosition(meters / Constants.ELEVATOR.encoderCountsToMeters);
  }

  public ELEVATOR.SETPOINT getElevatorState() {
    return desiredHeightState;
  }

  public boolean getElevatingState() {
    return !(Math.abs(getPercentOutput()) < 0.05);
  }

  public void setSetpointState(ELEVATOR.SETPOINT heightEnum) {
    desiredHeightState = heightEnum;
  }

  public void setDesiredPositionMeters(double meters) {
    m_desiredPositionMeters = meters;
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
    joystickInput = m_joystickY;
  }

  /*
   * Coast: Motor moving without power
   * Brake: Motor is kept in place
   */
  public void setElevatorNeutralMode(NeutralMode mode) {
    elevatorMotors[0].setNeutralMode(mode);
    elevatorMotors[1].setNeutralMode(mode);
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

  // Update elevator height using encoders and bottom limit switch
  public void updateElevatorHeight() {
    /* Uses limit switch to act as a baseline
     * to reset the sensor position and height to improve accuracy
     */
    // if (getElevatorLowerSwitch()) {
    //   setElevatorSensorPosition(0.0);
    // }
    elevatorHeight = getHeightMeters();
  }

  public Translation2d getElevatorField2dTranslation() {
    return new Translation2d(
        -getHeightMeters() * Math.cos(Constants.ELEVATOR.mountAngleRadians.getRadians())
            + centerOffset,
        0);
  }

  private void initShuffleboard() {
    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Elevator Sim", mech2d);
    }
    SmartDashboard.putData("Elevator Subsystem", this);

    var elevatorNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Elevator");

    kHeightPub = elevatorNtTab.getDoubleTopic("Height Meters").publish();
    kHeightInchesPub = elevatorNtTab.getDoubleTopic("Height Inches").publish();
    kDesiredHeightPub = elevatorNtTab.getDoubleTopic("Desired Height").publish();
    kEncoderCountsPub = elevatorNtTab.getDoubleTopic("Encoder Counts").publish();
    kDesiredStatePub = elevatorNtTab.getStringTopic("Desired State").publish();
    kPercentOutputPub = elevatorNtTab.getStringTopic("Percent Output").publish();
    kClosedLoopModePub = elevatorNtTab.getStringTopic("Closed-Loop Mode").publish();
    currentCommandStatePub = elevatorNtTab.getStringTopic("Current Command State").publish();

    elevatorNtTab.getDoubleTopic("kP").publish().set(kP);
    elevatorNtTab.getDoubleTopic("kI").publish().set(kI);
    elevatorNtTab.getDoubleTopic("kD").publish().set(kD);

    elevatorNtTab.getDoubleTopic("Max Vel").publish().set(maxVel);
    elevatorNtTab.getDoubleTopic("Max Accel").publish().set(maxAccel);
    elevatorNtTab.getDoubleTopic("kS").publish().set(kS);
    elevatorNtTab.getDoubleTopic("kV").publish().set(kV);
    elevatorNtTab.getDoubleTopic("kA").publish().set(kA);

    elevatorNtTab.getDoubleTopic("setpoint").publish().set(0);

    kPSub = elevatorNtTab.getDoubleTopic("kP").subscribe(kP);
    kISub = elevatorNtTab.getDoubleTopic("kI").subscribe(kI);
    kDSub = elevatorNtTab.getDoubleTopic("kD").subscribe(kD);

    kMaxVelSub = elevatorNtTab.getDoubleTopic("Max Vel").subscribe(maxVel);
    kMaxAccelSub = elevatorNtTab.getDoubleTopic("Max Accel").subscribe(maxAccel);
    kSSub = elevatorNtTab.getDoubleTopic("kS").subscribe(kS);
    kVSub = elevatorNtTab.getDoubleTopic("kV").subscribe(kV);
    kASub = elevatorNtTab.getDoubleTopic("kA").subscribe(kA);

    kSetpointSub = elevatorNtTab.getDoubleTopic("setpoint").subscribe(0);
  }

  public void updateShuffleboard() {
    SmartDashboard.putBoolean("Elevator Closed Loop", getClosedLoopState());
    SmartDashboard.putNumber("Elevator Height Inches", Units.metersToInches(getHeightMeters()));

    kHeightPub.set(getHeightMeters());
    kHeightInchesPub.set(Units.metersToInches(getHeightMeters()));
    kDesiredHeightPub.set(getDesiredPositionMeters());
    kEncoderCountsPub.set(getElevatorEncoderCounts());
    kDesiredStatePub.set(desiredHeightState.name());
    kPercentOutputPub.set(String.format("%.0f%%", getPercentOutput() * 100.0));
    kClosedLoopModePub.set(isClosedLoop ? "Closed" : "Open");

    currentCommandStatePub.set(getControlState().toString());

    // Elevator PID Tuning Values
    //    if (DriverStation.isTest()) {
    //      elevatorMotors[0].config_kP(0, kPSub.get(0));
    //      elevatorMotors[0].config_kI(0, kISub.get(0));
    //      elevatorMotors[0].config_kD(0, kDSub.get(0));
    //
    //      maxVel = kMaxVelSub.get(0);
    //      maxAccel = kMaxAccelSub.get(0);
    //      m_trapezoidialConstraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
    //      kS = kSSub.get(0);
    //      kV = kVSub.get(0);
    //      kA = kASub.get(0);
    //      m_feedForward = new SimpleMotorFeedforward(kS, kV, kA);
    //
    //      var testSetpoint = kSetpointSub.get(0);
    //      if (m_desiredPositionMeters != testSetpoint) {
    //        setControlState(ELEVATOR.STATE.SETPOINT);
    //        m_desiredPositionMeters = testSetpoint;
    //      }
    //    }
  }

  public void updateLog() {
    // elevatorCurrentEntry.append(getElevatorMotorVoltage());
    elevatorSetpointEntry.append(m_desiredPositionMeters);
    elevatorPositionEntry.append(elevatorHeight);
  }

  @Override
  public void simulationPeriodic() {
    elevatorSim.setInput(getPercentOutput() * 12);

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);

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
    // Yes, this needs to be called in the periodic. The simulation does not work without this
    updateShuffleboard();
    updateElevatorHeight();
    if (isClosedLoop) {
      switch (m_controlState) {
        case CLOSED_LOOP_MANUAL:
          m_desiredPositionMeters =
              MathUtil.clamp(
                  joystickInput * setpointMultiplier + getHeightMeters(),
                  ELEVATOR.THRESHOLD.ABSOLUTE_MIN.get(),
                  ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get());
          break;
        case OPEN_LOOP_MANUAL:
          double percentOutput = joystickInput * percentOutputMultiplier;
          if (getHeightMeters() > (getUpperLimitMeters() - 0.0254)) {
            percentOutput = Math.min(percentOutput, 0);
          }
          if (getHeightMeters() < (getLowerLimitMeters() + 0.005)) {
            percentOutput = Math.max(percentOutput, 0);
          }
          setPercentOutput(percentOutput);
          break;
        case USER_SETPOINT:
          m_desiredPositionMeters += joystickInput * setpointMultiplier;
          break;
        default:
        case AUTO_SETPOINT:
          break;
      }
      if (DriverStation.isEnabled() && m_controlState != ELEVATOR.STATE.OPEN_LOOP_MANUAL) {
        m_goal = new TrapezoidProfile.State(m_desiredPositionMeters, 0);
        var profile = new TrapezoidProfile(m_trapezoidialConstraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(0.02);
        //      var commandedSetpoint = limitDesiredAngleSetpoint();
        var commandedSetpoint = limitDesiredSetpointMeters(m_setpoint);
        m_commandedPositionMeters = commandedSetpoint.position;
        kDesiredHeightPub.set(Units.metersToInches(commandedSetpoint.position));
        setSetpointTrapezoidState(commandedSetpoint);
      }
    } else {
      // TODO: If targetElevatorLowerSwitch() is triggered, do not set a negative percent output
      double percentOutput = joystickInput * percentOutputMultiplier;
      if (getHeightMeters() > (getUpperLimitMeters() - 0.0254)) {
        percentOutput = Math.min(percentOutput, 0);
      }
      if (getHeightMeters() < (getLowerLimitMeters() + 0.0015)) {
        percentOutput = Math.max(percentOutput, 0);
      }
      setPercentOutput(percentOutput);
    }
  }
}
