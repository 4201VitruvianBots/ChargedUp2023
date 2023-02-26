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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.WRIST;

public class Wrist extends SubsystemBase {
  private double m_desiredSetpointRadians;
  private double m_commandedAngleRadians;
  private double m_lowerLimitRadians = WRIST.THRESHOLD.ABSOLUTE_MIN.get();
  private double m_upperLimitRadians = WRIST.THRESHOLD.ABSOLUTE_MAX.get();
  private boolean isClosedLoop = true;
  private WRIST.STATE m_controlState = WRIST.STATE.SETPOINT;
  private final boolean wristLowerLimitOverride = false;
  private double m_joystickInput;
  private double m_wristPercentOutput;

  private static final DigitalInput wristLowerSwitch =
      new DigitalInput(Constants.getInstance().Wrist.wristLowerSwitch);
  /** Creates a new Wrist. */
  private static final TalonFX wristMotor = new TalonFX(Constants.CAN.wristMotor);

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(600));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  // Create a new ArmFeedforward with gains kS, kG, kV, and kA
  public ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.getInstance().Wrist.FFkS,
          Constants.getInstance().Wrist.kG,
          Constants.getInstance().Wrist.FFkV,
          Constants.getInstance().Wrist.kA);

  private final double maxPercentOutput = 1.0;
  private final double setpointMultiplier = Units.degreesToRadians(60.0);
  private final double percentOutputMultiplier = 0.4;

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          Constants.getInstance().Wrist.gearBox,
          Constants.getInstance().Wrist.wristGearRatio,
          SingleJointedArmSim.estimateMOI(
              Constants.getInstance().Wrist.wristLength, Constants.getInstance().Wrist.wristMass),
          Constants.getInstance().Wrist.wristLength,
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
  public static ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");

  private final DoubleSubscriber kSSub;
  private final DoubleSubscriber kVSub;
  private final DoubleSubscriber kGSub;
  private final DoubleSubscriber kASub;
  private final DoubleSubscriber kPSub;
  private final DoubleSubscriber kDSub;
  private final DoubleSubscriber setpointSub;
  private DoublePublisher kSetpointTargetPub;

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

    wristMotor.config_kP(0, Constants.getInstance().Wrist.kP);
    wristMotor.config_kD(0, Constants.getInstance().Wrist.kD);
    wristMotor.configPeakOutputForward(
        maxPercentOutput, Constants.getInstance().Elevator.kTimeoutMs);
    wristMotor.configPeakOutputReverse(
        -maxPercentOutput, Constants.getInstance().Elevator.kTimeoutMs);

    wristMotor.setInverted(TalonFXInvertType.Clockwise);

    wristMotor.configAllowableClosedloopError(
        0, 1 / Constants.getInstance().Wrist.encoderUnitsToDegrees);
    Timer.delay(1);
    resetWristAngle(-10.0);

    wristTab.addDouble("Angle Degrees", this::getPositionDegrees);
    wristTab.addDouble("Raw Position", this::getSensorPosition);
    wristTab.addDouble(
        "Setpoint Degrees", () -> Units.radiansToDegrees(getDesiredPositionRadians()));
    wristTab.addDouble("Velocity DPS", this::getVelocityDegreesPerSecond);
    wristTab.addString("State", () -> getControlState().toString());
    wristTab.addBoolean("isClosedLoop", this::getControlMode);
    wristTab.add(this);

    try {
      NetworkTableInstance.getDefault()
          .getTable("Wrist")
          .getDoubleTopic("kA")
          .publish()
          .set(Constants.getInstance().Wrist.kA);
      NetworkTableInstance.getDefault()
          .getTable("Wrist")
          .getDoubleTopic("kS")
          .publish()
          .set(Constants.getInstance().Wrist.FFkS);
      NetworkTableInstance.getDefault()
          .getTable("Wrist")
          .getDoubleTopic("kV")
          .publish()
          .set(Constants.getInstance().Wrist.FFkV);
      NetworkTableInstance.getDefault()
          .getTable("Wrist")
          .getDoubleTopic("kG")
          .publish()
          .set(Constants.getInstance().Wrist.kG);
      NetworkTableInstance.getDefault()
          .getTable("Wrist")
          .getDoubleTopic("kP")
          .publish()
          .set(Constants.getInstance().Wrist.kP);
      NetworkTableInstance.getDefault()
          .getTable("Wrist")
          .getDoubleTopic("kD")
          .publish()
          .set(Constants.getInstance().Wrist.kD);
      NetworkTableInstance.getDefault()
          .getTable("Wrist")
          .getDoubleTopic("setpoint")
          .publish()
          .set(0);
      kSetpointTargetPub =
          NetworkTableInstance.getDefault()
              .getTable("Wrist")
              .getDoubleTopic("calculated setpoint")
              .publish();
    } catch (Exception e) {

    }
    kASub =
        NetworkTableInstance.getDefault()
            .getTable("Wrist")
            .getDoubleTopic("kA")
            .subscribe(Constants.getInstance().Wrist.kA);
    kSSub =
        NetworkTableInstance.getDefault()
            .getTable("Wrist")
            .getDoubleTopic("kS")
            .subscribe(Constants.getInstance().Wrist.FFkS);
    kVSub =
        NetworkTableInstance.getDefault()
            .getTable("Wrist")
            .getDoubleTopic("kV")
            .subscribe(Constants.getInstance().Wrist.FFkV);
    kGSub =
        NetworkTableInstance.getDefault()
            .getTable("Wrist")
            .getDoubleTopic("kG")
            .subscribe(Constants.getInstance().Wrist.kG);
    kPSub =
        NetworkTableInstance.getDefault()
            .getTable("Wrist")
            .getDoubleTopic("kP")
            .subscribe(Constants.getInstance().Wrist.kP);
    kDSub =
        NetworkTableInstance.getDefault()
            .getTable("Wrist")
            .getDoubleTopic("kD")
            .subscribe(Constants.getInstance().Wrist.kD);
    setpointSub =
        NetworkTableInstance.getDefault().getTable("Wrist").getDoubleTopic("setpoint").subscribe(0);

    if (RobotBase.isSimulation()) {
      //            wristMotor.setSensorPhase(true);
    }
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
  private void setWristPercentOutput(double value) {
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
  public void setSetpointDegrees(TrapezoidProfile.State state) {
    wristMotor.set(
        ControlMode.Position,
        Units.radiansToDegrees(state.position)
            / Constants.getInstance().Wrist.encoderUnitsToDegrees,
        DemandType.ArbitraryFeedForward,
        //                    0
        calculateFeedforward(state));
    m_wristPercentOutput = wristMotor.getMotorOutputPercent() + calculateFeedforward(state);
  }

  private double calculateFeedforward(TrapezoidProfile.State state) {
    return (m_feedforward.calculate(state.position, state.velocity) / 12.0);
  }

  public void resetState() {
    m_setpoint =
        new TrapezoidProfile.State(
            Units.degreesToRadians(getPositionDegrees()),
            Units.degreesToRadians(getVelocityDegreesPerSecond()));
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
    return getSensorPosition() * Constants.getInstance().Wrist.encoderUnitsToDegrees;
  }
  // this is get current angle
  public double getVelocityDegreesPerSecond() {
    return wristMotor.getSelectedSensorVelocity()
        * Constants.getInstance().Wrist.encoderUnitsToDegrees
        * 10;
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
    wristMotor.setSelectedSensorPosition(
        angle / Constants.getInstance().Wrist.encoderUnitsToDegrees); // setWristSensorPosition(0);
    //    if(RobotBase.isSimulation()) {
    //      wristMotor.getSimCollection().setIntegratedSensorRawPosition(
    //              (int) (angle
    //                      / Constants.getInstance().Wrist.encoderUnitsPerRotation));
    //    }
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

  // SmartDashboard function
  public void updateSmartDashboard() {}

  public void updateLog() {
    wristVoltageEntry.append(getWristMotorVoltage());
    wristCurrentEntry.append(getWristMotorCurrent());
    wristDesiredPositionEntry.append(getDesiredPositionRadians());
    wristCommandedPositionEntry.append(getCommandedPositionRadians());
    wristPositionDegreesEntry.append(getPositionDegrees());
  }

  @Override
  public void periodic() {
    var kS = kSSub.get(0);
    var kG = kSSub.get(0);
    var kV = kSSub.get(0);
    var kA = kSSub.get(0);
    var kP = kPSub.get(0);
    var kD = kDSub.get(0);

    m_feedforward = new ArmFeedforward(kS, kG, kV, kA);
    wristMotor.config_kP(0, kP);
    wristMotor.config_kD(0, kD);

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
        default:
        case SETPOINT:
          break;
      }
      if (DriverStation.isEnabled()) {
        m_goal = new TrapezoidProfile.State(m_desiredSetpointRadians, 0);
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(0.02);
        var commandedSetpoint = limitDesiredSetpointRadians(m_setpoint);
        m_commandedAngleRadians = commandedSetpoint.position;
        kSetpointTargetPub.set(Units.radiansToDegrees(commandedSetpoint.position));
        setSetpointDegrees(commandedSetpoint);
      }
    } else {
      setWristPercentOutput(m_joystickInput * percentOutputMultiplier);
    }
  }

  @Override
  public void simulationPeriodic() {
    m_armSim.setInputVoltage(
        MathUtil.clamp(m_wristPercentOutput * RobotController.getBatteryVoltage(), -12, 12));
    m_armSim.update(0.020);

    Unmanaged.feedEnable(20);

    // Using negative sensor units to match physical behavior
    wristMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            -(int)
                (Units.radiansToDegrees(m_armSim.getAngleRads())
                    / Constants.getInstance().Wrist.encoderUnitsToDegrees));

    wristMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            -(int)
                (Units.radiansToDegrees(m_armSim.getVelocityRadPerSec())
                    / Constants.getInstance().Wrist.encoderUnitsToDegrees
                    * 10.0));
  }
}
