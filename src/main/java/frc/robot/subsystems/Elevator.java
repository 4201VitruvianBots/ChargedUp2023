// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {

  // Initializing both motors
  public static final TalonFX[] elevatorMotors = {
    new TalonFX(Constants.Elevator.elevatorMotorLeft),
    new TalonFX(Constants.Elevator.elevatorMotorRight)
  };

  // Used by RobotContainer to specify which button has been pressed
  public enum elevatorHeights {
    LOW,
    MID,
    HIGH,
    JOYSTICK,
    NONE
  }

  // Limit switch at bottom of elevator
  private static DigitalInput elevatorLowerSwitch =
      new DigitalInput(Constants.Elevator.elevatorLowerSwitch);

  private static double
      desiredHeightValue; // The height in encoder units our robot is trying to reach
  private static elevatorHeights desiredHeightState =
      elevatorHeights.NONE; // Think of this as our "next state" in our state machine.

  private static double elevatorJoystickY;

  private final double kF = 0; // Only F and P control is needed for Elevator
  private final double kP = 0.2;

  private static double elevatorHeight =
      0; // the amount of rotations the motor has gone up from the initial low position

  // Simulation setup

  private static boolean isSimulated = false;

  private static final ElevatorSim elevatorSim =
      new ElevatorSim(
          Constants.Elevator.elevatorGearbox,
          Constants.Elevator.elevatorGearing,
          Constants.Elevator.elevatorMassKg,
          Constants.Elevator.elevatorDrumRadiusMeters,
          Constants.Elevator.elevatorMinHeightMeters,
          Constants.Elevator.elevatorMaxHeightMeters,
          true,
          VecBuilder.fill(0.01));

  // Shuffleboard setup

  public static ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

  public static GenericEntry elevatorHeightTab = elevatorTab.add("Elevator Height", 0.0).getEntry();
  public static GenericEntry elevatorTargetHeightTab =
      elevatorTab.add("Elevator Target Height", desiredHeightValue).getEntry();
  public static GenericEntry elevatorTargetPosTab =
      elevatorTab.add("Elevator Target Position", desiredHeightState.name()).getEntry();
  public static GenericEntry elevatorRawPerOutTab =
      elevatorTab.add("Elevator Raw Percent Output", 0.0).getEntry();
  public static GenericEntry elevatorPerOutTab =
      elevatorTab.add("Elevator Percent Output", "0%").getEntry();

  /* Constructs a new Elevator. Mostly motor setup */
  public Elevator() {
    for (TalonFX motor : elevatorMotors) {
      motor.configFactoryDefault();
      motor.setNeutralMode(NeutralMode.Brake);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(elevatorHeight);

    }

    elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());

    elevatorMotors[0].config_kF(0, kF);
    elevatorMotors[0].config_kP(0, kP);
    
    initShuffleboard();
    SmartDashboard.putData(this);
  }
  /*
   * Elevator's motor output as a percentage
   */
  public static double getElevatorPercentOutput() {
    return elevatorMotors[0].getMotorOutputPercent();
  }

  public static void setElevatorPercentOutput(double output) {
    elevatorMotors[0].set(ControlMode.PercentOutput, output);
  }

  /*
   * Elevator's height position
   */
  public static double getElevatorHeight() {
    return elevatorHeight;
  }

  public static void setElevatorHeight(double height) {
    elevatorHeight = height;
  }

  public double getElevatorMotorVoltage() {
    return elevatorMotors[0].getMotorOutputVoltage();
  }

  public static boolean getElevatorLowerSwitch() {
    return elevatorLowerSwitch.get();
  }

  public static boolean getElevatorSimulated() {
    return isSimulated;
  }

  public static void setElevatorSensorPosition(double position) {
    elevatorMotors[0].setSelectedSensorPosition(position);
  }

  public static elevatorHeights getElevatorDesiredHeightState() {
    return desiredHeightState;
  }

  public static void setElevatorDesiredHeightState(elevatorHeights heightEnum) {
    desiredHeightState = heightEnum;
  }

  public static void setElevatorJoystickY(DoubleSupplier m_joystickY) {
    elevatorJoystickY = m_joystickY.getAsDouble();
  }

  /*
   * Coast: Motor moving without power
   * Brake: Motor is kept in place
   */
  public void setElevatorNeutralMode(NeutralMode mode) {
    elevatorMotors[0].setNeutralMode(mode);
    elevatorMotors[1].setNeutralMode(mode);
  }

  public static void updateSimulatedElevatorHeight() {
    setElevatorHeight(getElevatorHeight()+(getElevatorPercentOutput()/10));
    //setElevatorHeight(elevatorSim.getPositionMeters());
  }

  // Update elevator height using encoders and bottom limit switch
  public static void updateElevatorHeight() {

    /* Uses limit switch to act as a baseline
     * to reset the sensor position and height to improve accuracy
     */
    if (getElevatorLowerSwitch()) {
      setElevatorHeight(0.0);
      setElevatorSensorPosition(0.0);
    } else {
      /* Uses built in feedback sensor if not at limit switch */
      setElevatorHeight(elevatorMotors[0].getSelectedSensorPosition());
    }
  }

  public static void initShuffleboard() {
    // TODO: Add encoder counts per second or since last scheduler run

    elevatorHeightTab.setDouble(getElevatorHeight());
    elevatorTargetHeightTab.setDouble(desiredHeightValue);
    elevatorTargetPosTab.setString(desiredHeightState.name());

    elevatorRawPerOutTab.setDouble(getElevatorPercentOutput());

    /* Converts the raw percent output to something more readable, by
     *  rounding it to the nearest whole number and turning it into an actual percentage.
     *  Example: -0.71247 -> -71%
     */
    elevatorPerOutTab.setString(String.valueOf(Math.round(getElevatorPercentOutput() * 100)) + "%");
    
  }

  @Override
  public void simulationPeriodic() {
    Elevator.isSimulated = true;

    elevatorSim.setInput(getElevatorMotorVoltage() * RobotController.getBatteryVoltage());

    elevatorSim.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
  }

  @Override
  public void periodic() {
    initShuffleboard();
    // This method will be called once per scheduler run
    if(desiredHeightState == elevatorHeights.JOYSTICK) {
      Elevator.setElevatorPercentOutput(elevatorJoystickY*0.8);
    }
    else {
      switch (desiredHeightState) {
        case LOW:
          desiredHeightValue = 0.0; // Placeholder values
        case MID:
          desiredHeightValue = 5.0; // Placeholder values
        case HIGH:
          desiredHeightValue = 10.0; // Placeholder values
        case NONE:
          desiredHeightValue = elevatorHeight;
      }
      double distanceBetween = desiredHeightValue - elevatorHeight;
      // Checking if our desired height has been reached within a certain range
      if (distanceBetween < 0.1 && distanceBetween > -0.1) { // Placeholder values
        setElevatorDesiredHeightState(elevatorHeights.NONE);
      }
      else {
        // TODO: Replace bang-bang controls with motion magic
        // The part where we actually determine where the elevator should move
        if (distanceBetween < 0) {
            Elevator.setElevatorPercentOutput(-0.8);
        } else if (distanceBetween > 0) {
          Elevator.setElevatorPercentOutput(0.8);
        } else if (distanceBetween == 0) {
          Elevator.setElevatorPercentOutput(0.0);
        }
      }
    }
  }
}
