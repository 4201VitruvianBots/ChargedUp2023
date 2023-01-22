// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private TalonFX wristMotor = new TalonFX(Constants.Wrist.wristMotor);
  public Wrist() {
    // One motor for the wrist 
    
    // factory default configs
    wristMotor.configFactoryDefault();
  }
  // control mode function
  public void setWristState(boolean state){
    boolean isWristtaking = state;
  }
    
  public boolean getWristState(boolean state){
    return state;
  }
  // set percent output function
  public void setWristPercentOutput(double value) {
    wristMotor.set(ControlMode.PercentOutput, value);
  }
  // shuffleboard funciton
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Wrist", getWristState(false));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}