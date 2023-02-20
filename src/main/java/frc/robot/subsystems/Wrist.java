// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private TalonFX wristMotor = new TalonFX(Constants.CAN.wristMotor);

  private boolean isWristtaking;
  private final double kF = 0;
  private final double kP = 0.2;

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

    wristMotor.setStatusFramePeriod(1, 255);
    wristMotor.setStatusFramePeriod(2, 255);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(10);
    wristMotor.enableVoltageCompensation(true);

    wristMotor.config_kF(0, kF);
    wristMotor.config_kP(0, kP);
  }

  public void setSetpoint(double setpoint) {}

  public double getMeasurement() {
    return wristMotor.getSelectedSensorPosition();
  }

  public boolean getWristState() {
    return isWristtaking;
  }

  public void setWristState(boolean state) {
    boolean isWristtaking = state;
  }

  public double getWristMotorVoltage() {
    return wristMotor.getMotorOutputVoltage();
  }

  // set percent output function
  public void setWristPercentOutput(double output) {
    wristMotor.set(ControlMode.PercentOutput, output);
  }
  // smartdashboard funciton
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Wrist", getWristState());
    SmartDashboard.putNumber("getWrist", 1);
  }

  public void updateLog() {
    wristCurrentEntry.append(getWristMotorVoltage());
    // wristSetpointEntry.append();
    wristPositionEntry.append(getMeasurement());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
