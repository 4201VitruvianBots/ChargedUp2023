// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WRIST;

public class Wrist extends SubsystemBase {
  private double m_desiredSetpointRadians;
  private double m_commandedAngleRadians;
  private double m_lowerLimitRadians = WRIST.THRESHOLD.ABSOLUTE_MIN.get();
  private double m_upperLimitRadians = WRIST.THRESHOLD.ABSOLUTE_MAX.get();
  private boolean isClosedLoop = true;
  private WRIST.STATE m_controlState = WRIST.STATE.AUTO_SETPOINT;
  private double m_joystickInput;
  private double m_simPercentOutput;
  private final int simEncoderSign =
      WRIST.motorInversionType == TalonFXInvertType.Clockwise ? -1 : 1;

  private static final DigitalInput wristLowerSwitch =
      new DigitalInput(Constants.DIO.wristLowerSwitch);
  /** Creates a new Wrist. */
  private static final TalonFX wristMotor = new TalonFX(Constants.CAN.wristMotor);

  private TrapezoidProfile.Constraints m_trapezoidalConstraints =
      new TrapezoidProfile.Constraints(Constants.WRIST.kMaxVel, Constants.WRIST.kMaxAccel);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  // Create a new ArmFeedforward with gains kS, kG, kV, and kA
  public ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.WRIST.FFkS, Constants.WRIST.kG, Constants.WRIST.FFkV, Constants.WRIST.kA);

  private final double maxPercentOutput = 1.0;
  public final double setpointMultiplier = Units.degreesToRadians(60.0);
  private final double percentOutputMultiplier = 0.4;

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          Constants.WRIST.gearBox,
          Constants.WRIST.gearRatio,
          SingleJointedArmSim.estimateMOI(Constants.WRIST.length, Constants.WRIST.mass),
          Constants.WRIST.length,
          WRIST.THRESHOLD.ABSOLUTE_MIN.get(),
          WRIST.THRESHOLD.ABSOLUTE_MAX.get(),
          false
          //          VecBuilder.fill(2.0 * Math.PI / 2048.0) // Add noise with a std-dev of 1 tick
          );
  // Logging setup

  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry wristVoltageEntry = new DoubleLogEntry(log, "/wrist/voltage");
  public DoubleLogEntry wristCurrentEntry = new DoubleLogEntry(log, "/wrist/current");
  public DoubleLogEntry wristDesiredPositionEntry =
      new DoubleLogEntry(log, "/wrist/desiredPositionDegrees");
  public DoubleLogEntry wristCommandedPositionEntry =
      new DoubleLogEntry(log, "/wrist/commandedPositionDegrees");
  public DoubleLogEntry wristPositionDegreesEntry =
      new DoubleLogEntry(log, "/wrist/positionDegrees");

  private DoubleSubscriber kPSub,
      kISub,
      kDSub,
      kMaxVelSub,
      kMaxAccelSub,
      kSSub,
      kGSub,
      kVSub,
      kASub,
      kSetpointSub;
  private DoublePublisher kCommandedAngleDegreesPub;
  private StringPublisher currentCommandStatePub;

  public Wrist() {
    // One motor for the wrist

    // factory default configs
    wristMotor.configFactoryDefault();
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    //    wristMotor.setStatusFramePeriod(1, 0);
    //    wristMotor.setStatusFramePeriod(2, 0);
    wristMotor.configVoltageCompSaturation(10);
    wristMotor.enableVoltageCompensation(true);

    wristMotor.config_kP(0, WRIST.kP);
    wristMotor.config_kD(0, WRIST.kD);
    wristMotor.configPeakOutputForward(maxPercentOutput, WRIST.kTimeoutMs);
    wristMotor.configPeakOutputReverse(-maxPercentOutput, WRIST.kTimeoutMs);

    wristMotor.setInverted(WRIST.motorInversionType);

    wristMotor.configAllowableClosedloopError(0, 1 / WRIST.encoderUnitsToDegrees);
    Timer.delay(1);
    resetWristAngle(90.0);

    initSmartDashboard();
  }

  public boolean getClosedLoopState() {
    return isClosedLoop;
  }

  public void setUserInput(double input) {
    m_joystickInput = input;
  }

  public void setControlState(WRIST.STATE state) {
    m_controlState = state;
  }

  public WRIST.STATE getControlState() {
    return m_controlState;
  }

  // set percent output function
  // period function that edits the elevator's height, from there make sure it obeys the limit (27.7
  // rotation)
  private void setPercentOutput(double value) {
    wristMotor.set(ControlMode.PercentOutput, value);
  }

  // code to limit the minimum/maximum setpoint of the wrist/ might be status frames
  public double getWristMotorVoltage() {
    return wristMotor.getMotorOutputVoltage();
  }

  public double getWristMotorCurrent() {
    return wristMotor.getSupplyCurrent();
  }

  //  setpoint for the wrist
  public void setSetpointTrapezoidState(TrapezoidProfile.State state) {
    wristMotor.set(
        ControlMode.Position,
        Units.radiansToDegrees(state.position) / Constants.WRIST.encoderUnitsToDegrees,
        DemandType.ArbitraryFeedForward,
        //                    0
        calculateFeedforward(state));
  }

  private double calculateFeedforward(TrapezoidProfile.State state) {
    return (m_feedforward.calculate(state.position, state.velocity) / 12.0);
  }

  public void resetState() {
    m_setpoint =
        new TrapezoidProfile.State(
            getPositionRadians(), Units.degreesToRadians(getVelocityDegreesPerSecond()));
  }

  public void setDesiredPositionRadians(double desiredAngleRadians) {
    m_desiredSetpointRadians = desiredAngleRadians;
  }

  public double getDesiredPositionRadians() {
    return m_desiredSetpointRadians;
  }

  public double getCommandedPositionRadians() {
    return m_commandedAngleRadians;
  }

  public double getPositionRadians() {
    return Units.degreesToRadians(getPositionDegrees());
  }

  // this is get current angle
  public double getPositionDegrees() {
    return getSensorPosition() * WRIST.encoderUnitsToDegrees;
  }
  // this is get current angle
  public double getVelocityDegreesPerSecond() {
    return wristMotor.getSelectedSensorVelocity() * WRIST.encoderUnitsToDegrees * 10;
  }

  public Rotation2d getWristAngleRotation2d() {
    return Rotation2d.fromDegrees(getPositionDegrees());
  }

  private double getSensorPosition() {
    return wristMotor.getSelectedSensorPosition();
  }

  // reset wrist angle function based off of a limit switch/hall effect sensor
  public void zeroEncoder() {
    // if (getLimitSwitchState(0)) {
    //   wristMotor.setSelectedSensorPosition(0, 0, 0);
    // } else if (getLimitSwitchState(1)) {
    //   wristMotor.setSelectedSensorPosition(0, 0, 0);
    // }
  }

  // TODO: Add limit switches
  private boolean getLimitSwitchState(int i) {
    return false;
  }
  //  public static boolean getWristLowerSwitch() {
  //    return !wristLowerSwitch.get();
  //  }

  // reset angle of the wrist. ~-15 degrees is the position of the wrist when the intake is touching
  // the ground.
  public void resetWristAngle(double angle) {
    wristMotor.setSelectedSensorPosition(angle / Constants.WRIST.encoderUnitsToDegrees);
  }

  public void setControlMode(boolean isClosedLoop) {
    this.isClosedLoop = isClosedLoop;
  }

  public boolean getControlMode() {
    return isClosedLoop;
  }

  public void setLowerLimit(double radians) {
    m_lowerLimitRadians = radians;
  }

  public double getLowerLimit() {
    return m_lowerLimitRadians;
  }

  public void setUpperLimit(double radians) {
    m_upperLimitRadians = radians;
  }

  public double getUpperLimit() {
    return m_upperLimitRadians;
  }

  //
  private TrapezoidProfile.State limitDesiredSetpointRadians(TrapezoidProfile.State state) {
    return new TrapezoidProfile.State(
        MathUtil.clamp(state.position, m_lowerLimitRadians, m_upperLimitRadians), state.velocity);
  }

  private void initSmartDashboard() {
    SmartDashboard.putData(this);

    var wristTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Wrist");

    wristTab.getDoubleTopic("kMaxVel").publish().set(Constants.WRIST.kMaxVel);
    wristTab.getDoubleTopic("kMaxAccel").publish().set(Constants.WRIST.kMaxAccel);
    wristTab.getDoubleTopic("kA").publish().set(Constants.WRIST.kA);
    wristTab.getDoubleTopic("kS").publish().set(Constants.WRIST.FFkS);
    wristTab.getDoubleTopic("kV").publish().set(Constants.WRIST.FFkV);
    wristTab.getDoubleTopic("kG").publish().set(Constants.WRIST.kG);
    wristTab.getDoubleTopic("kP").publish().set(Constants.WRIST.kP);
    wristTab.getDoubleTopic("kI").publish().set(Constants.WRIST.kI);
    wristTab.getDoubleTopic("kD").publish().set(Constants.WRIST.kD);
    wristTab.getDoubleTopic("Desired Angle Degrees").publish().set(0);

    kCommandedAngleDegreesPub = wristTab.getDoubleTopic("Commanded Angle Degrees").publish();
    currentCommandStatePub = wristTab.getStringTopic("Command State").publish();

    kMaxVelSub = wristTab.getDoubleTopic("kMaxVel").subscribe(Constants.WRIST.kMaxVel);
    kMaxAccelSub = wristTab.getDoubleTopic("kMaxAccel").subscribe(Constants.WRIST.kMaxAccel);
    kSSub = wristTab.getDoubleTopic("kS").subscribe(Constants.WRIST.FFkS);
    kGSub = wristTab.getDoubleTopic("kG").subscribe(Constants.WRIST.kG);
    kVSub = wristTab.getDoubleTopic("kV").subscribe(Constants.WRIST.FFkV);
    kASub = wristTab.getDoubleTopic("kA").subscribe(Constants.WRIST.kA);
    kPSub = wristTab.getDoubleTopic("kP").subscribe(Constants.WRIST.kP);
    kISub = wristTab.getDoubleTopic("kI").subscribe(Constants.WRIST.kI);
    kDSub = wristTab.getDoubleTopic("kD").subscribe(Constants.WRIST.kD);
    kSetpointSub = wristTab.getDoubleTopic("Desired Angle Degrees").subscribe(0);
  }

  // SmartDashboard function
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Wrist Closed Loop", getClosedLoopState());
    SmartDashboard.putNumber("Wrist Angles Degrees", getPositionDegrees());

    currentCommandStatePub.set(getControlState().toString());
    //    if (DriverStation.isTest()) {
    //      var maxVel = kMaxVelSub.get(0);
    //      var maxAccel = kMaxAccelSub.get(0);
    //      m_trapezoidalConstraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
    //      var kS = kSSub.get(Constants.WRIST.FFkS);
    //      var kG = kGSub.get(Constants.WRIST.kG);
    //      var kV = kVSub.get(Constants.WRIST.FFkV);
    //      var kA = kASub.get(Constants.WRIST.kA);
    //
    //      m_feedforward = new ArmFeedforward(kS, kG, kV, kA);
    //
    //      wristMotor.config_kP(0, kPSub.get(0));
    //      wristMotor.config_kI(0, kISub.get(0));
    //      wristMotor.config_kD(0, kDSub.get(0));
    //
    //      var testSetpoint = kSetpointSub.get(0);
    //      if (m_desiredSetpointRadians != testSetpoint) {
    //        setControlState(Constants.WRIST.STATE.SETPOINT);
    //        m_desiredSetpointRadians = testSetpoint;
    //      }
    //    }
  }

  public void updateLog() {
    wristVoltageEntry.append(getWristMotorVoltage());
    wristCurrentEntry.append(getWristMotorCurrent());
    wristDesiredPositionEntry.append(getDesiredPositionRadians());
    wristCommandedPositionEntry.append(getCommandedPositionRadians());
    wristPositionDegreesEntry.append(getPositionDegrees());
  }

  public boolean isScoring() {
    return (getPositionDegrees() > 170);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    updateLog();
    // This method will be called once per scheduler run
    if (isClosedLoop) {
      switch (m_controlState) {
        case CLOSED_LOOP_MANUAL:
          m_desiredSetpointRadians =
              MathUtil.clamp(
                  m_joystickInput * setpointMultiplier + getPositionRadians(),
                  WRIST.THRESHOLD.ABSOLUTE_MIN.get(),
                  WRIST.THRESHOLD.ABSOLUTE_MAX.get());
          break;
        case OPEN_LOOP_MANUAL:
          double percentOutput = m_joystickInput * percentOutputMultiplier;
          if (getPositionRadians() > (getUpperLimit() - 0.0523599)) {
            percentOutput = Math.min(percentOutput, 0);
          }
          if (getPositionRadians() < (getLowerLimit() + 0.0523599)) {
            percentOutput = Math.max(percentOutput, 0);
          }
          setPercentOutput(percentOutput);
          break;
        case USER_SETPOINT:
          m_desiredSetpointRadians += m_joystickInput * setpointMultiplier;
          break;
        default:
        case AUTO_SETPOINT:
          break;
      }
      if (DriverStation.isEnabled() && m_controlState != WRIST.STATE.OPEN_LOOP_MANUAL) {
        m_goal = new TrapezoidProfile.State(m_desiredSetpointRadians, 0);
        var profile = new TrapezoidProfile(m_trapezoidalConstraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(0.02);
        var commandedSetpoint = limitDesiredSetpointRadians(m_setpoint);
        m_commandedAngleRadians = commandedSetpoint.position;
        kCommandedAngleDegreesPub.set(Units.radiansToDegrees(commandedSetpoint.position));
        setSetpointTrapezoidState(commandedSetpoint);
      }
    } else {
      setPercentOutput(m_joystickInput * percentOutputMultiplier);
    }
  }

  @Override
  public void simulationPeriodic() {
    m_armSim.setInputVoltage(MathUtil.clamp(wristMotor.getMotorOutputPercent(), -12, 12));
    m_armSim.update(0.020);

    Unmanaged.feedEnable(20);

    // Using negative sensor units to match physical behavior
    wristMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (simEncoderSign
                    * Units.radiansToDegrees(m_armSim.getAngleRads())
                    / Constants.WRIST.encoderUnitsToDegrees));

    wristMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (simEncoderSign
                    * Units.radiansToDegrees(m_armSim.getVelocityRadPerSec())
                    / Constants.WRIST.encoderUnitsToDegrees
                    * 10.0));
  }
}
