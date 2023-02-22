// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Wrist.WRIST_STATE;

public class Wrist extends SubsystemBase {
  private WRIST_STATE m_desiredState = WRIST_STATE.JOYSTICK;
  private double desiredAngleSetpoint;
  private double m_lowerAngleLimitDegrees;
  private double m_upperAngleLimitDegrees;
  private boolean wristIsClosedLoop = false;
  private boolean wristLowerLimitOverride = false;
  private double m_joystickInput;

  private static DigitalInput wristLowerSwitch =
      new DigitalInput(Constants.getInstance().Wrist.wristLowerSwitch);
  /** Creates a new Wrist. */
  private static TalonFX wristMotor = new TalonFX(Constants.CAN.wristMotor);

  // Create a new ArmFeedforward with gains kS, kG, kV, and kA
  private ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.getInstance().Wrist.FFkS,
          Constants.getInstance().Wrist.kG,
          Constants.getInstance().Wrist.FFkV,
          Constants.getInstance().Wrist.kA);

  private double setpointMultiplier = 1;

  // Logging setup

  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry wristCurrentEntry = new DoubleLogEntry(log, "/wrist/wristCurrent");
  public DoubleLogEntry wristSetpointEntry = new DoubleLogEntry(log, "/wrist/wristSetpoint");
  public DoubleLogEntry wristPositionEntry = new DoubleLogEntry(log, "/wrist/wristPosition");
  public static ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");

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
    zeroWristAngle();

    wristMotor.configPeakOutputForward(0.25);
    wristMotor.configPeakOutputReverse(0.25);
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

  //  setpoint for the wrist
  public void setSetpointDegrees(double degrees) {
    wristMotor.set(
        ControlMode.Position,
        degrees / Constants.getInstance().Wrist.encoderUnitsPerRotation,
        DemandType.ArbitraryFeedForward,
        m_feedforward.calculate(
            degrees,
            wristMotor.getActiveTrajectoryVelocity()
                * Constants.getInstance().Wrist.encoderUnitsPerRotation // TODO: Verify this
                * 10));
  }

  public double getSetpointDegrees() {
    return desiredAngleSetpoint;
  }

  public void setWristState(WRIST_STATE state) {
    m_desiredState = state;
  }

  public WRIST_STATE getWristState() {
    return m_desiredState;
  }

  // this is get current angle
  public double getWristAngleDegrees() {
    return getWristSensorPosition() * Constants.getInstance().Wrist.encoderUnitsPerRotation;
  }

  public Rotation2d getWristAngleRotation2d() {
    return Rotation2d.fromDegrees(getWristAngleDegrees());
  }

  private double getWristSensorPosition() {
    return wristMotor.getSelectedSensorPosition();
  }

  // reset wrist angle function based off of a limit switch/hall effect sensor
  public void zeroEncoder() {
    if (getLimitSwitchState(0)) {
      wristMotor.setSelectedSensorPosition(0, 0, 0);
    } else if (getLimitSwitchState(1)) {
      wristMotor.setSelectedSensorPosition(0, 0, 0);
    }
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
  public void zeroWristAngle() {
    setSetpointDegrees(-15.0); // setWristSensorPosition(0);
  }

  public void setControlMode(boolean isClosedLoop) {
    wristIsClosedLoop = isClosedLoop;
  }

  public boolean getControlMode() {
    return wristIsClosedLoop;
  }

  public void setLowerAngleLimit(double angleDegrees) {
    m_lowerAngleLimitDegrees = angleDegrees;
  }

  public void setUpperAngleLimit(double angleDegrees) {
    m_upperAngleLimitDegrees = angleDegrees;
  }

  //
  private double limitDesiredAngleSetpoint() {
    return MathUtil.clamp(desiredAngleSetpoint, m_lowerAngleLimitDegrees, m_upperAngleLimitDegrees);
  }

  // SmartDashboard function
  public void updateSmartDashboard() {
    wristTab.addDouble("Angle", this::getWristAngleDegrees);
    wristTab.addDouble("Raw position", this::getWristSensorPosition);
    wristTab.addDouble("Setpoint", this::getSetpointDegrees);
  }

  public void updateLog() {
    wristCurrentEntry.append(getWristMotorVoltage());
    wristSetpointEntry.append(getSetpointDegrees());
    wristPositionEntry.append(getWristAngleDegrees());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    updateLog();
    // This method will be called once per scheduler run
    if (wristIsClosedLoop) {
      switch (m_desiredState) {
        case JOYSTICK:
          desiredAngleSetpoint = m_joystickInput * setpointMultiplier + getWristSensorPosition();
          break;
        case INTAKING:
          desiredAngleSetpoint = -10.0;
          break;
        case LOW:
          // TODO: Find setpoint value
          desiredAngleSetpoint = 0;
          break;
        case MID:
          // TODO: Find setpoint value
          desiredAngleSetpoint = 0;
          break;
        case HIGH:
          // TODO: Find setpoint value
          desiredAngleSetpoint = 0;
          break;
        default:
        case STOWED:
          desiredAngleSetpoint = 50.0;
          break;
      }
      setSetpointDegrees(limitDesiredAngleSetpoint());
    } else {
      setWristPercentOutput(m_joystickInput * setpointMultiplier);
    }
  }
}
