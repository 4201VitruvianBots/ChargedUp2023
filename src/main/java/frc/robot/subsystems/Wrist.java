// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
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
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class Wrist extends SubsystemBase {
  private double m_desiredAngleDegrees;
  private double m_commandedAngleDegrees;
  private double m_lowerLimitDegrees = Constants.getInstance().Wrist.wristAbsoluteLowerLimitDegrees;
  private double m_upperLimitDegrees = Constants.getInstance().Wrist.wristAbsoluteUpperLimitDegrees;
  private boolean wristIsClosedLoop = true;
  private boolean wristLowerLimitOverride = false;
  private double m_joystickInput;
  private double m_wristPercentOutput;

  private static DigitalInput wristLowerSwitch =
      new DigitalInput(Constants.getInstance().Wrist.wristLowerSwitch);
  /** Creates a new Wrist. */
  private static TalonFX wristMotor = new TalonFX(Constants.CAN.wristMotor);

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

  private double setpointMultiplier = 60.0;

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          Constants.getInstance().Wrist.gearBox,
          Constants.getInstance().Wrist.wristGearRatio,
          SingleJointedArmSim.estimateMOI(
              Constants.getInstance().Wrist.wristLength, Constants.getInstance().Wrist.wristMass),
          Constants.getInstance().Wrist.wristLength,
          Units.degreesToRadians(Constants.getInstance().Wrist.wristAbsoluteLowerLimitDegrees),
          Units.degreesToRadians(Constants.getInstance().Wrist.wristAbsoluteUpperLimitDegrees),
          true,
          VecBuilder.fill(2.0 * Math.PI / 2048.0) // Add noise with a std-dev of 1 tick
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

  private DoubleSubscriber kSSub, kVSub, kGSub, kASub, kPSub, kDSub, setpointSub;
  private DoublePublisher kSetpointPub;

  public Wrist() {
    // One motor for the wrist

    // factory default configs
    wristMotor.configFactoryDefault();

    wristMotor.setInverted(true);

    wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    wristMotor.setStatusFramePeriod(1, 0);
    wristMotor.setStatusFramePeriod(2, 0);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(10);
    wristMotor.enableVoltageCompensation(true);

    wristMotor.config_kP(0, Constants.getInstance().Wrist.kP);
    wristMotor.config_kD(0, Constants.getInstance().Wrist.kD);
    Timer.delay(1);
    resetWristAngle(0);

    wristMotor.configAllowableClosedloopError(
        0, 1 / Constants.getInstance().Wrist.encoderUnitsPerRotation);

    wristTab.addDouble("Angle", this::getAngleDegrees);
    wristTab.addDouble("Raw position", this::getWristSensorPosition);
    wristTab.addDouble("Setpoint", this::getDesiredAngle);
    wristTab.addDouble("Wrist Velocity", this::getWristAngleDegreesPerSecond);
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
      kSetpointPub =
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
  }

  public void setWristInput(double input) {
    m_joystickInput = input;
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
            / Constants.getInstance().Wrist.encoderUnitsPerRotation,
        DemandType.ArbitraryFeedForward,
        //                    0
        calculateFeedforward(state));
    m_wristPercentOutput = wristMotor.getMotorOutputPercent();
  }

  private double calculateFeedforward(TrapezoidProfile.State state) {
    return (m_feedforward.calculate(state.position, state.velocity) / 12.0);
  }

  public void resetState() {
    m_setpoint =
        new TrapezoidProfile.State(
            Units.degreesToRadians(getAngleDegrees()),
            Units.degreesToRadians(getWristAngleDegreesPerSecond()));
  }

  public void setDesiredAngle(double desiredAngleDegrees) {
    m_desiredAngleDegrees = desiredAngleDegrees;
  }

  public double getDesiredAngle() {
    return m_desiredAngleDegrees;
  }

  public double getCommandedAngle() {
    return m_commandedAngleDegrees;
  }

  // this is get current angle
  public double getAngleDegrees() {
    return getWristSensorPosition() * Constants.getInstance().Wrist.encoderUnitsPerRotation;
  }
  // this is get current angle
  public double getWristAngleDegreesPerSecond() {
    return wristMotor.getSelectedSensorVelocity()
        * Constants.getInstance().Wrist.encoderUnitsPerRotation
        * 10;
  }

  public Rotation2d getWristAngleRotation2d() {
    return Rotation2d.fromDegrees(getAngleDegrees());
  }

  private double getWristSensorPosition() {
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
        angle
            / Constants.getInstance().Wrist.encoderUnitsPerRotation); // setWristSensorPosition(0);
  }

  public void setControlMode(boolean isClosedLoop) {
    wristIsClosedLoop = isClosedLoop;
  }

  public boolean getControlMode() {
    return wristIsClosedLoop;
  }

  public void setLowerLimit(double angleDegrees) {
    m_lowerLimitDegrees = angleDegrees;
  }

  public double getLowerLimit() {
    return m_lowerLimitDegrees;
  }

  public void setUpperLimit(double angleDegrees) {
    m_upperLimitDegrees = angleDegrees;
  }

  public double getUpperLimit() {
    return m_upperLimitDegrees;
  }

  //
  private TrapezoidProfile.State limitDesiredAngleSetpoint(TrapezoidProfile.State state) {
    return new TrapezoidProfile.State(
        MathUtil.clamp(state.position, Units.degreesToRadians(m_lowerLimitDegrees), Units.degreesToRadians(m_upperLimitDegrees)), state.velocity);
  }

  // SmartDashboard function
  public void updateSmartDashboard() {}

  public void updateLog() {
    wristVoltageEntry.append(getWristMotorVoltage());
    wristCurrentEntry.append(getWristMotorCurrent());
    wristDesiredPositionEntry.append(getDesiredAngle());
    wristCommandedPositionEntry.append(getCommandedAngle());
    wristPositionDegreesEntry.append(getAngleDegrees());
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
    if (wristIsClosedLoop) {
      m_desiredAngleDegrees = m_joystickInput * setpointMultiplier + getAngleDegrees();
      //      switch (m_desiredAngleDegrees) {
      //        case JOYSTICK:
      //          desiredAngleSetpoint = m_joystickInput * setpointMultiplier + getAngleDegrees();
      //          break;
      //        case INTAKING:
      //          desiredAngleSetpoint = -10.0;
      //          break;
      //        case LOW:
      //          // TODO: Find setpoint value
      //          desiredAngleSetpoint = 35;
      //          break;
      //        case MID:
      //          // TODO: Find setpoint value
      //          desiredAngleSetpoint = 0;
      //          break;
      //        case HIGH:
      //          // TODO: Find setpoint value
      //          desiredAngleSetpoint = 0;
      //          break;
      //        default:
      //        case STOWED:
      //          desiredAngleSetpoint = 50.0;
      //          break;
      //      }
      if (DriverStation.isEnabled()) {
        //        desiredAngleSetpoint = setpointSub.get(0);
        m_goal = new TrapezoidProfile.State(Units.degreesToRadians(m_desiredAngleDegrees), 0);
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(0.02);
        var commandedSetpoint = limitDesiredAngleSetpoint(m_setpoint);
        m_commandedAngleDegrees = m_setpoint.position;
        kSetpointPub.set(Units.radiansToDegrees(m_commandedAngleDegrees));
        setSetpointDegrees(commandedSetpoint);
      }
    } else {
      setWristPercentOutput(m_joystickInput * setpointMultiplier);
    }
  }

  @Override
  public void simulationPeriodic() {
    m_armSim.setInput(m_wristPercentOutput * RobotController.getBatteryVoltage());

    m_armSim.update(0.020);

    Unmanaged.feedEnable(20);

    wristMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (Units.radiansToDegrees(m_armSim.getAngleRads())
                    / Constants.getInstance().Wrist.encoderUnitsPerRotation));

    wristMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (Units.radiansToDegrees(m_armSim.getVelocityRadPerSec())
                    / Constants.getInstance().Wrist.encoderUnitsPerRotation
                    * 10.0));
  }
}
