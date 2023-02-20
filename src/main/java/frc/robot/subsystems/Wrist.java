// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Wrist extends SubsystemBase {
  private static final double maxRotationValue = Constants.constants.Wrist.wristmaxRotationDegrees;
  private static final int encoderCountsPerAngle = 0;
  private static final boolean wristLowerLimitOverride = false;
  private static double desiredRotationValue;
  private static WristRotations desiredRotationState = WristRotations.NONE;
  private double wristPosition = 0;
  private static double wristJoystickX;

  public enum WristRotations {
    LOW,
    MEDIUM,
    JOYSTICK,
    NONE
  }

  private static DigitalInput wristLowerSwitch =
      new DigitalInput(Constants.constants.Wrist.wristLowerSwitch);
  /** Creates a new Wrist. */
  private static TalonFX wristMotor = new TalonFX(Constants.constants.Wrist.wristMotor);

  private boolean isWristMoving;
  private final double kF = 0;
  private final double kP = 0.2;
  private double setpointMultiplier = 1;

  // Logging setup

  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry wristCurrentEntry = new DoubleLogEntry(log, "/wrist/wristCurrent");
  public DoubleLogEntry wristSetpointEntry = new DoubleLogEntry(log, "/elevator/wristSetpoint");
  public DoubleLogEntry wristPositionEntry = new DoubleLogEntry(log, "/elevator/wristPosition");

  public Wrist() {
    // One motor for the wrist

    // factory default configs
    wristMotor.configFactoryDefault();

    wristMotor.setInverted(false);

    wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    wristMotor.setStatusFramePeriod(1, 0);
    wristMotor.setStatusFramePeriod(2, 0);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(10);
    wristMotor.enableVoltageCompensation(true);

    wristMotor.config_kF(0, kF);
    wristMotor.config_kP(0, kP);
  }
  //  setpoint for the wrist
  public void setSetpoint(double setpoint) {
    this.desiredRotationValue = setpoint;
  }

  public boolean getWristState() {
    return isWristMoving;
  }

  public void setWristState(boolean state) {
    isWristMoving = state;
  }

  public double getDegrees(int motorIndex) {
    return wristMotor.getSelectedSensorVelocity()
        * (360.0 / Constants.constants.Wrist.encoderUnitsPerRotation)
        / Constants.constants.Wrist.wristGearRatio;
  }
  // this is get current angle
  public double getAngle() {
    return getWristPosition() / encoderCountsPerAngle;
  }

  public void zeroEncoder() {
    if (getLimitSwitchState(0)) {
      wristMotor.setSelectedSensorPosition(kP, 0, 0);
      isWristMoving = true;
    } else if (getLimitSwitchState(1)) {
      wristMotor.setSelectedSensorPosition(kF, 0, 0);
      isWristMoving = true;
    } else isWristMoving = false;
  }

  private boolean getLimitSwitchState(int i) {
    return false;
  }

  public void setEncoderPosition(int position) {
    wristMotor.setSelectedSensorPosition(position, 0, 0);
  }
  // reset angle of the wrist
  // set sensor to 0
  public void ResetWrist() {
    setSetpoint(desiredRotationValue); // setWristSensorPosition(0);
  }
  // code to limit the minimum/maximum setpoint of the wrist/ might be status frames
  public double getWristMotorVoltage() {
    return wristMotor.getMotorOutputVoltage();
  }

  // set percent output function
  // period function that edits the elevators height, from there make sure it obeys the limit (27.7
  // rotation)
  public void setWristPercentOutput(double value) {
    wristMotor.set(ControlMode.PercentOutput, value);
    wristPosition = getWristPosition();
    if (value > 0) {
      if (wristPosition < Constants.constants.Wrist.wristEncoderUpperLimit) {
        if (Math.abs(wristPosition - Constants.constants.Wrist.wristEncoderUpperLimit)
            < Constants.constants.Wrist.wristEncoderSlowdown) {
          wristMotor.set(
              ControlMode.PercentOutput,
              Math.min(value, Constants.constants.Wrist.maxSpeedLimitsPercent)
                  * Math.abs(wristPosition - Constants.constants.Wrist.wristEncoderUpperLimit)
                  / Constants.constants.Wrist.wristEncoderSlowdown);
        } else {
          wristMotor.set(ControlMode.PercentOutput, value);
        }
      } else wristMotor.set(ControlMode.PercentOutput, 0);
    } else {
      if (!wristLowerLimitOverride) {
        wristMotor.set(ControlMode.PercentOutput, 0);
      } else {
        wristMotor.set(ControlMode.PercentOutput, value);
      }
    }
  }

  private double getWristPosition() {
    return wristMotor.getSelectedSensorPosition();
  }
  // reset wrist angle function based off of a limit switch/hall effect sensor
  public static void updateWristRotation() {
    if (getWristLowerSwitch()) {
      setWristSensorPosition(0.0);
    }
  }

  public static void setWristSensorPosition(double position) {
    wristMotor.setSelectedSensorPosition(position);
  }

  public static boolean getWristLowerSwitch() {
    return !wristLowerSwitch.get();
  }
  // smartdashboard funciton
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Wrist", getWristState());
    SmartDashboard.putNumber("getWrist", 1);
  }

  public void updateLog() {
    wristCurrentEntry.append(getWristMotorVoltage());
    // wristSetpointEntry.append();
    wristPositionEntry.append(getWristPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (desiredRotationState) {
      case JOYSTICK:
        desiredRotationValue = wristJoystickX * setpointMultiplier + getWristPosition();
      case LOW:
        desiredRotationValue = 0.0; // Placeholder values
        break;
      case MEDIUM:
        desiredRotationValue = maxRotationValue / 2; // Placeholder values
        break;
      case NONE:
        break;
    }
    double distanceBetween =
    MathUtil.applyDeadband(
        desiredRotationValue - getWristPosition(), maxRotationValue / 10);
    if (distanceBetween == 0) {
      setWristPercentOutput(0.0);
    } else if (distanceBetween > 0) {
      setWristPercentOutput(0.2);
    } else if (distanceBetween < 0) {
      setWristPercentOutput(-0.2);
    }
  }

  public Object getWristDesiredRotationState() {
    return desiredRotationValue;
  }

  public static void setWristJoystickX(double m_JoystickX) {
    wristJoystickX = m_JoystickX;
  }

  public Object getWristDesiredRotations() {
    return desiredRotationState;
  }

  public void setWristDesiredRotationState(WristRotations wristEnum) {
    desiredRotationState = wristEnum;
  }
}
