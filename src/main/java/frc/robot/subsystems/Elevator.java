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
  /** Creates a new Elevator. */
  // Initialize 2 elevator motors as an array
  public final TalonFX[] elevatorMotors = {
    new TalonFX(Constants.Elevator.elevatorMotorLeft), new TalonFX(Constants.Elevator.elevatorMotorRight)
  };
  // 0 = left motor, 1 = right motor
  // We will likely use TalonFX motors
  private final double kF = 0;
  private final double kP = 0.2;
  // Delcare constants for F and P for PIDF control
  private boolean elevatorClimbState;

  // Boolean for elevator climb state, maybe double for holdPosition? (not sure what that is)
  // Int for desired height, 0 = no desired height/ground, 1 = middle nodes, 2 = top nodes

  public Elevator() {
    // Config both motors using a for loop on the array
    for(TalonFX motor : elevatorMotors){
      motor.configFactoryDefault();
      motor.setSelectedSensorPosition(0);
      motor.setNeutralMode(NeutralMode.Brake);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1));
    }

    elevatorMotors[0].setInverted(true);
    elevatorMotors[1].setInverted(true);
    // Invert both motors
    // Follower statement for right motor to follow left motor speed
    elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());
    // Config F and P for only the left motor for now cause thats how its done in Carbon for some reason, change it later if necessary
    elevatorMotors[0].config_kF(0, kF);
    elevatorMotors[0].config_kP(0, kP);
    // Set status frame period for both motors, frame values 1 and 2
    elevatorMotors[0].setStatusFramePeriod(1, 100);
    elevatorMotors[0].setStatusFramePeriod(2, 100);

    elevatorMotors[1].setStatusFramePeriod(1, 255);
    elevatorMotors[1].setStatusFramePeriod(2, 255);
  }

  // Function for setting both motors into neutral mode
  public void setElevatorNeutralMode(NeutralMode mode) {
    elevatorMotors[0].setNeutralMode(mode);
    elevatorMotors[1].setNeutralMode(mode);
  }

  public void setElevatorPercentOutput(double output) {
    elevatorMotors[0].set(ControlMode.PercentOutput, output);
  }
  // Function for setting the percent output of the motors

  public double getElevatorMotorVoltage() {
    return elevatorMotors[0].getMotorOutputVoltage();
  }

  public double getElevatorPercentOutput() {
    return elevatorMotors[0].getMotorOutputPercent();
  }

  // Getter methods for motor current, voltage, and output, input motor index
  // Setter methods for motor voltage and output

  /** Getter methods for position, height, and velocity
   * Position and velocity will check for both motor's health
  */

  // Encoder counts to inches conversion function
  public double rotationsToInches(double rotations) {
    return rotations*1;
  }

  public double inchesToRotations(double inches) {
    return inches*1;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
