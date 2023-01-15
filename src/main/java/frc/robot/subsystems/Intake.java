// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static final String Controlmode = null;
/** Creates a new Intake. */
  private boolean isIntaking = false; 
  private TalonFX intakeMotor = new TalonFX(Constants.Intake.intakeMotor);
  public Intake() {
    intakeMotor.configFactoryDefault();
    //factory default cofigs 
    // one or two motors
    
  }
  public void setIntakeSate(boolean state){
    isIntaking = state;
  }
  public void setIntakePercentOutput(double value){
    intakeMotor.set(ControlMode.PercentOutput, value);
  }

  // control mode function 
  // set percent output function
  // shuffleboard or smartdashboard funciton


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
