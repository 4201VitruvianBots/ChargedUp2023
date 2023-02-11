// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  // Initializing both motors
  public static final TalonFX[] elevatorMotors = {
    new TalonFX(Constants.Elevator.elevatorMotorLeft),
    new TalonFX(Constants.Elevator.elevatorMotorRight)
  };

  // Used by RobotContainer to specify which button has been pressed
  public enum elevatorHeights {
    STOWED,
    LOW,
    MID,
    HIGH,
    JOYSTICK,
  }

  // Limit switch at bottom of elevator
  private static DigitalInput elevatorLowerSwitch =
      new DigitalInput(Constants.Elevator.elevatorLowerSwitch);

  private static double
      desiredHeightValue; // The height in encoder units our robot is trying to reach
  private static elevatorHeights desiredHeightState =
      elevatorHeights.STOWED; // Think of this as our "next state" in our state machine.

  private static double elevatorJoystickY;

  private final double kP = 0.55;
  private final double kI = 0;
  private final double kD = 0;
  private final double kF = 0.01;

  private static double elevatorHeight =
      0; // the amount of rotations the motor has gone up from the initial low position

  private static final double maxElevatorHeight = Constants.Elevator.elevatorMaxHeightMeters;

  // By default this is set to true as we use motion magic to determine what speed we should be at
  // to get to our setpoint.
  // If the sensors are acting up, we set this value to false to directly control the percent output
  // of the motors.
  private boolean elevatorIsClosedLoop = true;

  // Simulation setup

  private static final ElevatorSim elevatorSim =
      new ElevatorSim(
          Constants.Elevator.elevatorGearbox,
          Constants.Elevator.elevatorGearing,
          Constants.Elevator.elevatorMassKg,
          Constants.Elevator.elevatorDrumRadiusMeters,
          Constants.Elevator.elevatorMinHeightMeters,
          Constants.Elevator.elevatorMaxHeightMeters,
          true);

  // Shuffleboard setup

  public static ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

  public GenericEntry elevatorHeightTab = elevatorTab.add("Elevator Height", 0.0).getEntry();
  public GenericEntry elevatorTargetHeightTab =
      elevatorTab.add("Elevator Target Height", desiredHeightValue).getEntry();
  public GenericEntry elevatorTargetPosTab =
      elevatorTab.add("Elevator Target Position", desiredHeightState.name()).getEntry();
  public GenericEntry elevatorRawPerOutTab =
      elevatorTab.add("Elevator Raw Percent Output", 0.0).getEntry();
  public GenericEntry elevatorPerOutTab =
      elevatorTab.add("Elevator Percent Output", "0%").getEntry();
  public GenericEntry elevatorControlLoopTab =
      elevatorTab.add("Elevator Control Loop", "Closed").getEntry();

  // Mechanism2d visualization setup

  public Mechanism2d mech2d = new Mechanism2d(maxElevatorHeight * 50, maxElevatorHeight * 50);
  public MechanismRoot2d root2d = mech2d.getRoot("Elevator", maxElevatorHeight * 25, 0);
  public MechanismLigament2d elevatorLigament2d =
      root2d.append(new MechanismLigament2d("Elevator", elevatorHeight, 90));

  /* Constructs a new Elevator. Mostly motor setup */
  public Elevator() {
    for (TalonFX motor : elevatorMotors) {
      motor.configFactoryDefault();
      motor.setNeutralMode(NeutralMode.Brake);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(elevatorHeight);

      // Config PID
      motor.selectProfileSlot(Constants.Elevator.kSlotIdx, Constants.Elevator.kPIDLoopIdx);
      motor.config_kF(Constants.Elevator.kSlotIdx, kF, Constants.Elevator.kTimeoutMs);
      motor.config_kP(Constants.Elevator.kSlotIdx, kP, Constants.Elevator.kTimeoutMs);
      motor.config_kI(Constants.Elevator.kSlotIdx, kI, Constants.Elevator.kTimeoutMs);
      motor.config_kD(Constants.Elevator.kSlotIdx, kD, Constants.Elevator.kTimeoutMs);

      motor.configPeakOutputForward(1, Constants.Elevator.kTimeoutMs);
      motor.configPeakOutputReverse(-1, Constants.Elevator.kTimeoutMs);

      motor.setSensorPhase(
          true); // Forward direction = positive, forward velocity = positive, positive x positive =
      // positive

      motor.configMotionCruiseVelocity(15000, Constants.Elevator.kTimeoutMs);
      motor.configMotionAcceleration(6000, Constants.Elevator.kTimeoutMs);

      motor.setSelectedSensorPosition(0.0); // Zero both motors
    }

    elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());

    SmartDashboard.putData(this);
    SmartDashboard.putData("Elevator", mech2d);
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

  public static void setElevatorMotionMagicMeters(double setpoint) {
    elevatorMotors[0].set(
        TalonFXControlMode.MotionMagic, setpoint / Constants.Elevator.metersToEncoderCounts);
  }

  /*
   * Elevator's height position
   */
  public static double getElevatorHeight() {
    return elevatorMotors[0].getSelectedSensorPosition() * Constants.Elevator.metersToEncoderCounts;
  }

  public double getElevatorMotorVoltage() {
    return elevatorMotors[0].getMotorOutputVoltage();
  }

  public static boolean getElevatorLowerSwitch() {
    return !elevatorLowerSwitch.get();
  }

  public static void setElevatorSensorPosition(double position) {
    elevatorMotors[0].setSelectedSensorPosition(position);
  }

  public static elevatorHeights getElevatorDesiredHeightState() {
    return desiredHeightState;
  }

  public boolean getElevatingState() {
    return !(Math.abs(getElevatorPercentOutput()) < 0.05);
  }

  public void setElevatorDesiredHeightState(elevatorHeights heightEnum) {
    desiredHeightState = heightEnum;
  }

  public void setElevatorJoystickY(double m_joystickY) {
    elevatorJoystickY = m_joystickY;
  }

  /*
   * Coast: Motor moving without power
   * Brake: Motor is kept in place
   */
  public void setElevatorNeutralMode(NeutralMode mode) {
    elevatorMotors[0].setNeutralMode(mode);
    elevatorMotors[1].setNeutralMode(mode);
  }

  /*
   * Closed loop (default): Uses motion magic and a set setpoint to determine motor output
   * Open loop: Changes motor output directly
   */
  public void setElevatorControlLoop(boolean isClosedLoop) {
    elevatorIsClosedLoop = isClosedLoop;
  }

  public boolean getElevatorControlLoop() {
    return elevatorIsClosedLoop;
  }

  // Update elevator height using encoders and bottom limit switch
  public static void updateElevatorHeight() {
    /* Uses limit switch to act as a baseline
     * to reset the sensor position and height to improve accuracy
     */
    if (getElevatorLowerSwitch()) {
      setElevatorSensorPosition(0.0);
    }
    elevatorHeight = getElevatorHeight();
  }

  public void updateShuffleboard() {
    // TODO: Add encoder counts per second or since last scheduler run

    elevatorHeightTab.setDouble(getElevatorHeight());
    elevatorTargetHeightTab.setDouble(Elevator.desiredHeightValue);
    elevatorTargetPosTab.setString(desiredHeightState.name());

    elevatorRawPerOutTab.setDouble(getElevatorPercentOutput());

    /* Converts the raw percent output to something more readable, by
     *  rounding it to the nearest whole number and turning it into an actual percentage.
     *  Example: -0.71247 -> -71%
     */
    elevatorPerOutTab.setString(String.valueOf(Math.round(getElevatorPercentOutput() * 100)) + "%");

    if (elevatorIsClosedLoop) {
      elevatorControlLoopTab.setString("Closed");
    } else {
      elevatorControlLoopTab.setString("Open");
    }
  }

  @Override
  public void simulationPeriodic() {
    elevatorSim.setInput(getElevatorPercentOutput() * 12);

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);

    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int) (elevatorSim.getPositionMeters() / Constants.Elevator.metersToEncoderCounts));

    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (elevatorSim.getVelocityMetersPerSecond()
                    / Constants.Elevator.metersToEncoderCounts
                    * 10));

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    elevatorLigament2d.setLength(Units.metersToInches(elevatorSim.getPositionMeters()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard(); // Yes, this needs to be called in the periodic. The simulation does not
    // work without this
    updateElevatorHeight();
    switch (desiredHeightState) {
      case JOYSTICK:
        if (elevatorIsClosedLoop) {
          desiredHeightValue = elevatorHeight + (-elevatorJoystickY * maxElevatorHeight);
          break;
        } else {
          setElevatorPercentOutput(-elevatorJoystickY * 0.8);
          return;
        }
      case STOWED:
        desiredHeightValue = 0.0;
        break;
      case LOW:
        desiredHeightValue = maxElevatorHeight * 0.25; // Placeholder values
        break;
      case MID:
        desiredHeightValue = maxElevatorHeight * 0.5; // Placeholder values
        break;
      case HIGH:
        desiredHeightValue = maxElevatorHeight * 0.75; // Placeholder values
        break;
    }
    if (elevatorIsClosedLoop) {
      setElevatorMotionMagicMeters(desiredHeightValue);
    } else {
      double distanceBetween =
          MathUtil.applyDeadband(desiredHeightValue - elevatorHeight, maxElevatorHeight / 100);
      if (distanceBetween == 0) {
        setElevatorPercentOutput(0.0);
      } else if (distanceBetween > 0) {
        setElevatorPercentOutput(0.8);
      } else if (distanceBetween < 0) {
        setElevatorPercentOutput(-0.8);
      }
    }
  }
}
