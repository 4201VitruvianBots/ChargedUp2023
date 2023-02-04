// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
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

    wristMotor.setStatusFramePeriod(1, 255);
    wristMotor.setStatusFramePeriod(2, 255);
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

  // set percent output function
  public void setWristPercentOutput(double output) {
    wristMotor.set(ControlMode.PercentOutput, output);
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
}
