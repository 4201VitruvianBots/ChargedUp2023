// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private final double KP = 0.2;
  // Delcare constants for F and P for PIDF control
  private boolean elevatorClimbState;

  // Boolean for elevator climb state, maybe double for holdPosition? (not sure what that is)
  // Int for desired height, 0 = no desired height/ground, 1 = middle nodes, 2 = top nodes

  public Elevator() {
    // Config both motors using a for loop on the array
    for(TalonFX motor : elevatorMotors){
      motor.configFactoryDefault();
      motor.setSelectedSensorPosition(0);
      motor.setNeutralMode(NeutralMode.brake)
    }
    elevatorMotors[0].setInverted(true);
    elevatorMotors[1].setInverted(true);
    // Invert both motors
    // Follower statement for right motor to follow left motor speed
    // Config F and P for only the left motor for now cause thats how its done in Carbon for some reason, change it later if necessary
    // Set status frame period for both motors, frame values 1 and 2
  }

  // Function for setting both motors into neutral mode
  // Function for setting the percent output of the motors
  // Getter methods for motor current, voltage, and output, input motor index
  // Setter methods for motor voltage and output

  /** Getter methods for position, height, and velocity
   * Position and velocity will check for both motor's health
  */

  // Rotations to inches conversion function

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
