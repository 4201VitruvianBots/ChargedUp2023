// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  public final TalonFX[] elevatorMotors = {
    new TalonFX(Constants.Elevator.elevatorMotorLeft), new TalonFX(Constants.Elevator.elevatorMotorRight)
  };

  private final double kF = 0;
  private final double kP = 0.2;

  private boolean elevatorClimbState;

  public Elevator() {
    for(TalonFX motor : elevatorMotors){
      motor.configFactoryDefault();
      motor.setNeutralMode(NeutralMode.Brake);
    }

    elevatorMotors[0].setInverted(true);
    elevatorMotors[1].setInverted(true);

    elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());

    elevatorMotors[0].config_kF(0, kF);
    elevatorMotors[0].config_kP(0, kP);

    elevatorMotors[0].setStatusFramePeriod(1, 100);
    elevatorMotors[0].setStatusFramePeriod(2, 100);

    elevatorMotors[1].setStatusFramePeriod(1, 255);
    elevatorMotors[1].setStatusFramePeriod(2, 255);
  }

  public void setElevatorNeutralMode(NeutralMode mode) {
    elevatorMotors[0].setNeutralMode(mode);
    elevatorMotors[1].setNeutralMode(mode);
  }

  public boolean getElevatorClimbState() {
    return elevatorClimbState;
  }

  public void setElevatorClimbState(boolean climbState) {
    this.elevatorClimbState = climbState;
  }

  public void setElevatorPercentOutput(double output) {
    elevatorMotors[0].set(ControlMode.PercentOutput, output);
  }

  public double getElevatorPercentOutput() {
    return elevatorMotors[0].getMotorOutputPercent();
  }

  public double getElevatorMotorVoltage() {
    return elevatorMotors[0].getMotorOutputVoltage();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
