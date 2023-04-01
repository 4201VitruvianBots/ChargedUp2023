// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_UTIL_LIMIT;
import frc.robot.Constants.WRIST;
import frc.robot.Constants.WRIST.WRIST_SPEED;
import frc.robot.commands.wrist.ResetAngleDegrees;

public class Wrist extends SubsystemBase implements AutoCloseable {
  private double m_desiredSetpointRadians;
  private double m_commandedAngleRadians;
  private double m_lowerLimitRadians = WRIST.THRESHOLD.ABSOLUTE_MIN.get();
  private double m_upperLimitRadians = WRIST.THRESHOLD.ABSOLUTE_MAX.get();
  private WRIST.STATE m_controlState = WRIST.STATE.CLOSED_LOOP;

  private double currentKI = 0;
  private double newKI = 0;

  private double m_joystickInput;
  private boolean m_userSetpoint;

  // TODO: Make this universal/put in StateHandler
  private CAN_UTIL_LIMIT limitCanUtil = CAN_UTIL_LIMIT.NORMAL;

  private Translation2d m_wristHorizontalTranslation = new Translation2d();

  private final Intake m_intake;

  private static final DigitalInput lowerSwitch = new DigitalInput(Constants.DIO.wristLowerSwitch);

  /** Creates a new Wrist. */
  private static final TalonFX wristMotor = new TalonFX(Constants.CAN.wristMotor);

  private TrapezoidProfile.Constraints m_currentConstraints =
    Constants.WRIST.slowConstraints;

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  // Create a new ArmFeedforward with gains kS, kG, kV, and kA
  public ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.WRIST.FFkS, Constants.WRIST.kG, Constants.WRIST.FFkV, Constants.WRIST.kA);

  private final Timer m_timer = new Timer();
  private double m_lastTimestamp = 0;
  private double m_lastSimTimestamp = 0;

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
  public DoubleLogEntry voltageEntry = new DoubleLogEntry(log, "/wrist/voltage");
  public DoubleLogEntry currentEntry = new DoubleLogEntry(log, "/wrist/current");
  public DoubleLogEntry desiredPositionEntry =
      new DoubleLogEntry(log, "/wrist/desiredPositionDegrees");
  public DoubleLogEntry commandedPositionEntry =
      new DoubleLogEntry(log, "/wrist/commandedPositionDegrees");
  public DoubleLogEntry positionDegreesEntry = new DoubleLogEntry(log, "/wrist/positionDegrees");

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
  private DoublePublisher kDesiredAngleDegreesPub;
  private DoublePublisher currentTrapezoidVelocity;
  private DoublePublisher currentTrapezoidAcceleration;
  private StringPublisher currentCommandStatePub;

  public Wrist(Intake intake) {
    m_intake = intake;

    // factory default configs
    wristMotor.configFactoryDefault();
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    wristMotor.config_kP(0, WRIST.kP);
    wristMotor.config_kI(0, WRIST.kI);
    wristMotor.config_kD(0, WRIST.kD);
    wristMotor.configPeakOutputForward(Constants.WRIST.kMaxPercentOutput, WRIST.kTimeoutMs);
    wristMotor.configPeakOutputReverse(-Constants.WRIST.kMaxPercentOutput, WRIST.kTimeoutMs);

    // TODO: Review limits, test to see what is appropriate or not
    wristMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 30, 0.2));

    wristMotor.setInverted(WRIST.motorInversionType);

    wristMotor.configAllowableClosedloopError(0, 1 / WRIST.encoderUnitsToDegrees);
    if (RobotBase.isReal()) Timer.delay(2);

    resetAngleDegrees(-15.0);

    initSmartDashboard();
    m_timer.reset();
    m_timer.start();
  }

  public void setUserInput(double input) {
    m_joystickInput = input;
  }

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }

  public boolean isUserControlled() {
    return m_joystickInput != 0 && m_userSetpoint == false;
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
  private void setPercentOutput(double output, boolean enforceLimits) {
    if (enforceLimits) {
      if (getPositionRadians() > (getUpperLimit() - Units.inchesToMeters(1))) {
        output = Math.min(output, 0);
      }
      if (getPositionRadians() < (getLowerLimit() + 0.005)) {
        output = Math.max(output, 0);
      }
    }

    wristMotor.set(ControlMode.PercentOutput, output);
  }

  // code to limit the minimum/maximum setpoint of the wrist/ might be status frames
  public double getMotorOutputVoltage() {
    return wristMotor.getMotorOutputVoltage();
  }

  public double getMotorOutputCurrent() {
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

  public void haltPosition() {
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

  public void setReduceCanUtilization(CAN_UTIL_LIMIT limitCan) {
    limitCanUtil = limitCan;
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
  public void resetAngleDegrees(double angleDegrees) {
    wristMotor.setSelectedSensorPosition(angleDegrees / Constants.WRIST.encoderUnitsToDegrees);
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

  public void updateTrapezoidProfileConstraints(WRIST_SPEED speed) {
    switch (speed) {
      case FAST:
        m_currentConstraints = Constants.WRIST.fastConstraints;
        break;
      default:
      case SLOW:
        m_currentConstraints = Constants.WRIST.slowConstraints;
        break;
    }
  }

  // TODO: Is this necessary? If not, remove it
  private void updateIValue() {
    if (getPositionRadians() < Units.degreesToRadians(30)) {
      newKI = 0.00001;
    } else if (getPositionRadians() >= Units.degreesToRadians(30)) {
      newKI = 0;
    }
    if (currentKI != newKI) {
      wristMotor.config_kI(0, newKI);
      currentKI = newKI;
    }
  }

  private TrapezoidProfile.State limitDesiredSetpointRadians(TrapezoidProfile.State state) {
    return new TrapezoidProfile.State(
        MathUtil.clamp(state.position, m_lowerLimitRadians, m_upperLimitRadians), state.velocity);
  }

  // TODO: Add wristAngleDegrees to Wrist Tab
  private void initSmartDashboard() {
    SmartDashboard.putData(this);
    SmartDashboard.putData("Reset90", new ResetAngleDegrees(this, Units.degreesToRadians(90)));

    NetworkTable wristTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Wrist");

    wristTab.getDoubleTopic("kMaxVel").publish().set(Constants.WRIST.kMaxSlowVel);
    wristTab.getDoubleTopic("kMaxAccel").publish().set(Constants.WRIST.kMaxSlowAccel);
    wristTab.getDoubleTopic("kA").publish().set(Constants.WRIST.kA);
    wristTab.getDoubleTopic("kS").publish().set(Constants.WRIST.FFkS);
    wristTab.getDoubleTopic("kV").publish().set(Constants.WRIST.FFkV);
    wristTab.getDoubleTopic("kG").publish().set(Constants.WRIST.kG);
    wristTab.getDoubleTopic("kP").publish().set(Constants.WRIST.kP);
    wristTab.getDoubleTopic("kI").publish().set(Constants.WRIST.kI);
    wristTab.getDoubleTopic("kD").publish().set(Constants.WRIST.kD);
    wristTab.getDoubleTopic("Desired Angle Degrees").publish().set(0);

    // Initialize Test Values
    kCommandedAngleDegreesPub = wristTab.getDoubleTopic("Commanded Angle Degrees").publish();
    kDesiredAngleDegreesPub = wristTab.getDoubleTopic("Desired Angle Degrees").publish();
    currentCommandStatePub = wristTab.getStringTopic("Command State").publish();
    currentTrapezoidAcceleration = wristTab.getDoubleTopic("Trapezoid Acceleration").publish();
    currentTrapezoidVelocity = wristTab.getDoubleTopic("Trapezoid Velocity").publish();

    kMaxVelSub = wristTab.getDoubleTopic("kMaxSlowVel").subscribe(Constants.WRIST.kMaxSlowVel);
    kMaxAccelSub =
        wristTab.getDoubleTopic("kMaxSlowAccel").subscribe(Constants.WRIST.kMaxSlowAccel);
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
    SmartDashboard.putString("Wrist Closed Loop", getControlState().name());
    SmartDashboard.putNumber("Wrist Angles Degrees", getPositionDegrees());

    // currentTrapezoidAcceleration.set(m_currentTrapezoidalConstraints.maxAcceleration);
    // currentTrapezoidVelocity.set(m_currentTrapezoidalConstraints.maxVelocity);

    currentCommandStatePub.set(getControlState().toString());
    // kDesiredAngleDegreesPub.set(Units.radiansToDegrees(getDesiredPositionRadians()));

    // Tuning controls
    //    setControlState(STATE.TEST_SETPOINT);
    // if (m_controlState == STATE.TEST_SETPOINT) {
    //   DriverStation.reportWarning("USING WRIST TEST MODE!", false);
    //   var maxVel = kMaxVelSub.get(0);
    //   var maxAccel = kMaxAccelSub.get(0);
    //   m_currentTrapezoidalConstraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
    //   var kS = kSSub.get(Constants.WRIST.FFkS);
    //   var kG = kGSub.get(Constants.WRIST.kG);
    //   var kV = kVSub.get(Constants.WRIST.FFkV);
    //   var kA = kASub.get(Constants.WRIST.kA);

    //   m_feedforward = new ArmFeedforward(kS, kG, kV, kA);

    //   var newTestKP = kPSub.get(0);
    //   if (testKP != newTestKP) {
    //     wristMotor.config_kP(0, newTestKP);
    //     testKP = newTestKP;
    //   }
    //   var newTestKI = kISub.get(0);
    //   if (testKI != newTestKI) {
    //     wristMotor.config_kI(0, newTestKI);
    //     testKI = newTestKI;
    //   }
    //   var newTestKD = kDSub.get(0);
    //   if (testKD != newTestKD) {
    //     wristMotor.config_kD(0, newTestKD);
    //     testKD = newTestKD;
    //   }
    // }
  }

  public void updateLog() {
    voltageEntry.append(getMotorOutputVoltage());
    currentEntry.append(getMotorOutputCurrent());
    desiredPositionEntry.append(getDesiredPositionRadians());
    commandedPositionEntry.append(getCommandedPositionRadians());
    positionDegreesEntry.append(getPositionDegrees());
  }

  // TODO: Is this needed? If not, remove it
  public boolean isScoring() {
    return (getPositionDegrees() > 170);
  }

  // TODO: Do Not Remove. WIP for auto scoring
  public void updateHorizontalTranslation() {
    // Cube: f(x)=0.00000874723*t^3-0.00218403*t^2-0.101395*t+16;
    // Cone: f(x)=0.000860801*t^2-0.406027*t+16.3458;
    // Cube: f(x)=0.00000913468*t^3-0.00232508*t^2-0.0894341*t+16.1239;
    // Cone: f(x)=-0.270347*t+16.8574;
    double horizontalDistance = 0;
    if (m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CUBE)
      horizontalDistance =
          0.00000913468 * Math.pow(getPositionDegrees(), 3)
              - 0.00232508 * Math.pow(getPositionDegrees(), 2)
              - 0.0894341 * getPositionDegrees()
              + 16.1239;
    else if (m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE)
      horizontalDistance =
          //          0.00860801 * Math.pow(getPositionDegrees(), 2) +
          -0.270347 * getPositionDegrees() + 16.8574;
    m_wristHorizontalTranslation = new Translation2d(Units.inchesToMeters(horizontalDistance), 0);
  }

  public Translation2d getHorizontalTranslation() {
    return m_wristHorizontalTranslation;
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    updateLog();
    updateIValue();

    switch (m_controlState) {
      // Called when setting to open loop
      case OPEN_LOOP_MANUAL:
        double percentOutput = m_joystickInput * Constants.WRIST.kPercentOutputMultiplier;
        // Sets final percent output
        // True means it will enforce limits. In this way it is not truly open loop, but it'll prevent the robot from breaking
        setPercentOutput(percentOutput, true);
        break;
      default:
      case CLOSED_LOOP:
        // Updates our trapezoid profile state based on the time since our last periodic and our
        // recorded change in height
        m_goal = new TrapezoidProfile.State(m_desiredSetpointRadians, 0);
        TrapezoidProfile profile = new TrapezoidProfile(m_currentConstraints, m_goal, m_setpoint);
        double currentTime = m_timer.get();
        m_setpoint = profile.calculate(currentTime - m_lastTimestamp);
        m_lastTimestamp = currentTime;

        TrapezoidProfile.State commandedSetpoint = limitDesiredSetpointRadians(m_setpoint);
        m_commandedAngleRadians = commandedSetpoint.position;
        kDesiredAngleDegreesPub.set(commandedSetpoint.position);
        setSetpointTrapezoidState(commandedSetpoint);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    m_armSim.setInputVoltage(
        MathUtil.clamp(
            wristMotor.getMotorOutputPercent() * RobotController.getBatteryVoltage(), -12, 12));
    double currentTime = m_timer.get();
    m_armSim.update(currentTime - m_lastSimTimestamp);
    m_lastSimTimestamp = currentTime;

    Unmanaged.feedEnable(20);

    // Using negative sensor units to match physical behavior
    wristMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (Constants.WRIST.simEncoderSign
                    * Units.radiansToDegrees(m_armSim.getAngleRads())
                    / Constants.WRIST.encoderUnitsToDegrees));

    wristMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (Constants.WRIST.simEncoderSign
                    * Units.radiansToDegrees(m_armSim.getVelocityRadPerSec())
                    / Constants.WRIST.encoderUnitsToDegrees
                    * 10.0));
  }

  @Override
  public void close() throws Exception {
    lowerSwitch.close();
  }
}
