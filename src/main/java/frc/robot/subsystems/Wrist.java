// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private static final int encoderCountsPerAngle = 0;
  private static final boolean wristLowerLimitOverride = false;
  private double wristPosition = 0;

  public enum WristRotations {
    LOW,
    MEDIUM,
    JOYSTICK, 
    NONE
  }
  /** Creates a new Wrist. */
  private TalonFX wristMotor = new TalonFX(Constants.Wrist.wristMotor);

  private boolean isWristtaking;
  private final double kF = 0;
  private final double kP = 0.2;

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

  public void setSetpoint(double setpoint) {

  }

  public double getMeasurement() {
    return wristMotor.getSelectedSensorPosition();
  }

  public boolean getWristState() {
    return isWristtaking;
  }

  public void setWristState(boolean state) {
    boolean isWristtaking = state;
  }

  public double getDegrees(int motorIndex) {
    return wristMotor.getSelectedSensorVelocity()
        * (360.0 / Constants.Wrist.encoderUnitsPerRotation)
        / Constants.Wrist.gearRatio;
  }

  public double getAngle() {
    return getPosition() / encoderCountsPerAngle;
  }

  private int getPosition() {
    return 0;
  }

  public void zeroEncoder() {
    if(getLimitSwitchState(0)) {
      wristMotor.setSelectedSensorPosition(kP, 0, 0);
      isWristtaking = true;
  } else if(getLimitSwitchState(1)) {
      wristMotor.setSelectedSensorPosition(kF, 0, 0);
      isWristtaking = true;
  } else
      isWristtaking = false;
}

  private boolean getLimitSwitchState(int i) {
    return false;
  }

  public void setEncoderPosition(int position) {
    wristMotor.setSelectedSensorPosition(position, 0, 0);
}

  public void ResetWrist() {
    setSetpoint(0);
  }

  // set percent output function
  public void setWristPercentOutput(double value) {
    wristMotor.set(ControlMode.PercentOutput, value);
    wristPosition = getWristPosition();
    if (value > 0) {
      if (wristPosition < Constants.Wrist.wristEncoderUpperLimit) {
        if (Math.abs(wristPosition - Constants.Wrist.wristEncoderUpperLimit)
            < Constants.Wrist.wristEncoderSlowdown) {
          wristMotor.set(
              ControlMode.PercentOutput,
              Math.min(value, Constants.Wrist.maxSpeedLimitsPercent)
                  * Math.abs(wristPosition - Constants.Wrist.wristEncoderUpperLimit)
                  / Constants.Wrist.wristEncoderSlowdown);
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

  // smartdashboard funciton
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Wrist", getWristState());
    SmartDashboard.putNumber("getWrist", 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Object getWristDesiredHeights() {
    return null;
  }

  public static void setWristJoystickX(DoubleSupplier m_JoystickX) {
  }

  public Object getWristDesiredRotations() {
    return null;
  }

public void setWristDesiredRotationState(WristRotations joystick) {
}
}
