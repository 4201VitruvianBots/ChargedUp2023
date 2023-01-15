// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  public static final TalonFX[] elevatorMotors = {
    new TalonFX(Constants.Elevator.elevatorMotorLeft), new TalonFX(Constants.Elevator.elevatorMotorRight)
  };
  
  private static DigitalInput elevatorLowerSwitch = new DigitalInput(Constants.Elevator.elevatorLowerSwitch);

  public enum elevatorHeights {
    LOW,
    MID,
    HIGH,
    JOYSTICK
  }

  private final double kF = 0;
  private final double kP = 0.2;

  private static boolean elevatorClimbState;

  private static double elevatorHeight = 0; // the amount of rotations the motor has gone up from the initial low position

  /* Constructs a new Elevator. */
  public Elevator() {
    for(TalonFX motor : elevatorMotors){
      motor.configFactoryDefault();
      motor.setNeutralMode(NeutralMode.Brake);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(elevatorHeight);
    }

    elevatorMotors[0].setInverted(true);
    elevatorMotors[1].setInverted(true);

    elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());

    elevatorMotors[0].config_kF(0, kF);
    elevatorMotors[0].config_kP(0, kP);

  }

  public void setElevatorNeutralMode(NeutralMode mode) {
    elevatorMotors[0].setNeutralMode(mode);
    elevatorMotors[1].setNeutralMode(mode);
  }

  public static boolean getElevatorClimbState() {
    return elevatorClimbState;
  }

  public static void setElevatorClimbState(boolean climbState) {
    elevatorClimbState = climbState;
  }

  public static void setElevatorPercentOutput(double output) {
    elevatorMotors[0].set(ControlMode.PercentOutput, output);
  }

  public double getElevatorPercentOutput() {
    return elevatorMotors[0].getMotorOutputPercent();
  }

  public double getElevatorMotorVoltage() {
    return elevatorMotors[0].getMotorOutputVoltage();
  }
  
  public static double getElevatorHeight() {
    return elevatorHeight;
  }

  public static void setElevatorHeight(double height) {
    elevatorHeight = height;
  }
  
  public static boolean getElevatorLowerSwitch() {
    return elevatorLowerSwitch.get();
  }

  public static void setElevatorSensorPosition(double position) {
    elevatorMotors[0].setSelectedSensorPosition(position);
  }

  // TODO: Get elevator height from encoders or limit switches

  public static double moveToElevatorHeight(elevatorHeights heightEnum, double joystickY) {
    double desiredHeight = elevatorHeight; // Set to elevator height intially in case none of the switch states change it
    switch(heightEnum) {
      case JOYSTICK:
        desiredHeight = elevatorHeight+joystickY;
      case LOW:
        desiredHeight = 0.0; // Placeholder values
      case MID:
        desiredHeight = 5.0; // Placeholder values
      case HIGH:
        desiredHeight = 10.0; // Placeholder values
    }
    double distanceBetween = desiredHeight-elevatorHeight;
    if(distanceBetween == 0) {
      setElevatorClimbState(false);
    }
    else {
      setElevatorClimbState(true);
    }
    return distanceBetween;
  }

  // Update elevator height using encoders and bottom limit switch
  public static void updateElevatorHeight() {
    if(getElevatorLowerSwitch()) {
      setElevatorHeight(0.0);
      setElevatorSensorPosition(0.0);
    }
    else {
      setElevatorHeight(
        elevatorMotors[0].getSelectedSensorPosition()
      );
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
