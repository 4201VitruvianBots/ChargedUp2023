// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private TalonFX wristMotor = new TalonFX(Constants.Wrist.wristMotor);

  private final PIDController wristPID = new PIDController(0.1, 0.01, 0.05);
  private boolean isWristtaking;

  public Wrist() {
    // One motor for the wrist

    // factory default configs
    wristMotor.configFactoryDefault();
    wristPID.setTolerance(0.01);
    wristPID.enableContinuousInput(-180, 180);

    wristMotor.setInverted(false);

    wristMotor.setStatusFramePeriod(1, 255);
    wristMotor.setStatusFramePeriod(2, 255);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(10);
    wristMotor.enableVoltageCompensation(true);
  }

  public void setSetpoint(double setpoint) {
    wristPID.setSetpoint(setpoint);
  }

  public double getMeasurement() {
    return wristMotor.getSelectedSensorPosition();
  }

  public boolean getWristState(boolean state) {
    return isWristtaking;
  }

  // control mode function
  public boolean getWristState;

  public void setWristState(boolean state) {
    boolean isWristtaking = state;
  }
  // set percent output function
  public void setWristPercentOutput(double output) {
    wristMotor.set(ControlMode.PercentOutput, output);
  }
  // smartdashboard funciton
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Wrist", getWristState(false));
    SmartDashboard.putNumber("getWrist", 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setWristPercentOutput(wristPID.calculate(getMeasurement()));
  }
public Object getWristState() {
    return isWristtaking;
}
}