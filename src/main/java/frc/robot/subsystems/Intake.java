// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static final String Controlmode = null;
  /** Creates a new Intake. */
  private boolean isIntaking = false; 
  
  private TalonFX intakeMotor = new TalonFX(Constants.Intake.intakeMotor);
  public Intake() {
    // one motor for the intake 

    intakeMotor.configFactoryDefault();
    //factory default cofigs
  }

  public boolean getIntakeState() {
    return isIntaking;
  }
  // control mode function 
  public void setIntakeState(boolean state){
    isIntaking = state;
    
  }
  // set percent output function 
  public void setIntakePercentOutput(double value){
    intakeMotor.set(ControlMode.PercentOutput, value);
  }
    // shuffleboard or smartdashboard funciton
    public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Intake", getIntakeState());
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
