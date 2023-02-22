// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Elevator.ELEVATOR_STATE;

public class Elevator extends SubsystemBase {

  // Initializing both motors
  public static final TalonFX[] elevatorMotors = {
    new TalonFX(Constants.CAN.elevatorMotorLeft), new TalonFX(Constants.CAN.elevatorMotorRight)
  };

  // Limit switch at bottom of elevator
  private static DigitalInput elevatorLowerSwitch =
      new DigitalInput(Constants.DIO.elevatorLowerSwitch);

  private static double
      desiredHeightValue; // The height in encoder units our robot is trying to reach
  private ELEVATOR_STATE desiredHeightState =
      ELEVATOR_STATE.STOWED; // Think of this as our "next state" in our state machine.
  private double m_lowerLimitMeters = 0;
  private double m_upperLimitMeters = 0;

  private static double elevatorJoystickY;

  private final double kP = 0.55;
  private final double kI = 0;
  private final double kD = 0;
  private final double kF = 0.01;

  private static double elevatorHeight =
      0; // the amount of meters the motor has gone up from the initial stowed position

  private static final double maxElevatorHeight =
      Constants.getInstance().Elevator.elevatorMaxHeightMeters;

  // By default this is set to true as we use motion magic to determine what speed we should be at
  // to get to our setpoint.
  // If the sensors are acting up, we set this value to false to directly control the percent output
  // of the motors.
  private boolean elevatorIsClosedLoop = false;

  private int openLoopDeadband = 10;
  private double maxPercentOutput = 0.25;
  private double setpointMultiplier = 0.35;

  // Simulation setup

  private static final ElevatorSim elevatorSim =
      new ElevatorSim(
          Constants.getInstance().Elevator.elevatorGearbox,
          Constants.getInstance().Elevator.elevatorGearing,
          Constants.getInstance().Elevator.elevatorMassKg,
          Constants.getInstance().Elevator.elevatorDrumRadiusMeters,
          Constants.getInstance().Elevator.elevatorMinHeightMeters,
          Constants.getInstance().Elevator.elevatorMaxHeightMeters,
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
  public GenericEntry elevatorEncoderCountsTab =
      elevatorTab.add("Elevator Encoder Counts", 0.0).getEntry();

  // Mechanism2d visualization setup

  public Mechanism2d mech2d = new Mechanism2d(maxElevatorHeight * 50, maxElevatorHeight * 50);
  public MechanismRoot2d root2d = mech2d.getRoot("Elevator", maxElevatorHeight * 25, 0);
  public MechanismLigament2d elevatorLigament2d =
      root2d.append(new MechanismLigament2d("Elevator", elevatorHeight, 90));

  // Logging setup

  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry elevatorCurrentEntry = new DoubleLogEntry(log, "/elevator/elevatorCurrent");
  public DoubleLogEntry elevatorSetpointEntry =
      new DoubleLogEntry(log, "/elevator/elevatorSetpoint");
  public DoubleLogEntry elevatorPositionEntry =
      new DoubleLogEntry(log, "/elevator/elevatorPosition");

  /* Constructs a new Elevator. Mostly motor setup */
  public Elevator() {
    for (TalonFX motor : elevatorMotors) {
      motor.configFactoryDefault();
      motor.setNeutralMode(NeutralMode.Brake);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      motor.setSelectedSensorPosition(elevatorHeight);

      // Config PID
      motor.selectProfileSlot(
          Constants.getInstance().Elevator.kSlotIdx, Constants.getInstance().Elevator.kPIDLoopIdx);
      motor.config_kF(
          Constants.getInstance().Elevator.kSlotIdx,
          kF,
          Constants.getInstance().Elevator.kTimeoutMs);
      motor.config_kP(
          Constants.getInstance().Elevator.kSlotIdx,
          kP,
          Constants.getInstance().Elevator.kTimeoutMs);
      motor.config_kI(
          Constants.getInstance().Elevator.kSlotIdx,
          kI,
          Constants.getInstance().Elevator.kTimeoutMs);
      motor.config_kD(
          Constants.getInstance().Elevator.kSlotIdx,
          kD,
          Constants.getInstance().Elevator.kTimeoutMs);

      motor.configPeakOutputForward(1, Constants.getInstance().Elevator.kTimeoutMs);
      motor.configPeakOutputReverse(-1, Constants.getInstance().Elevator.kTimeoutMs);

      motor.configMotionCruiseVelocity(15000, Constants.getInstance().Elevator.kTimeoutMs);
      motor.configMotionAcceleration(6000, Constants.getInstance().Elevator.kTimeoutMs);

      motor.setSelectedSensorPosition(0.0); // Zero both motors

      motor.configPeakOutputForward(maxPercentOutput, 0);
    }

    elevatorMotors[1].set(TalonFXControlMode.Follower, elevatorMotors[0].getDeviceID());

    elevatorMotors[0].setInverted(TalonFXInvertType.CounterClockwise);
    elevatorMotors[1].setInverted(TalonFXInvertType.OpposeMaster);

    SmartDashboard.putData("Elevator Command", this);
    SmartDashboard.putData("Elevator", mech2d);
  }
  /*
   * Elevator's motor output as a percentage
   */
  public double getElevatorPercentOutput() {
    return elevatorMotors[0].getMotorOutputPercent();
  }

  public void setElevatorPercentOutput(double output) {
    elevatorMotors[0].set(ControlMode.PercentOutput, output);
  }

  public void setElevatorMotionMagicMeters(double setpoint) {
    elevatorMotors[0].set(
        TalonFXControlMode.MotionMagic,
        setpoint / Constants.getInstance().Elevator.metersToEncoderCounts);
  }

  /*
   * Elevator's height position
   */
  public double getHeightMeters() {
    return elevatorMotors[0].getSelectedSensorPosition()
        * Constants.getInstance().Elevator.metersToEncoderCounts;
  }

  public double getElevatorEncoderCounts() {
    return elevatorMotors[0].getSelectedSensorPosition();
  }

  public double getElevatorMotorVoltage() {
    return elevatorMotors[0].getMotorOutputVoltage();
  }

  public boolean getElevatorLowerSwitch() {
    return !elevatorLowerSwitch.get();
  }

  public void setElevatorSensorPosition(double position) {
    elevatorMotors[0].setSelectedSensorPosition(position);
  }

  public ELEVATOR_STATE getElevatorState() {
    return desiredHeightState;
  }

  public boolean getElevatingState() {
    return !(Math.abs(getElevatorPercentOutput()) < 0.05);
  }

  public void setElevatorState(ELEVATOR_STATE heightEnum) {
    desiredHeightState = heightEnum;
  }

  public void setLowerLimit(double meters) {
    m_lowerLimitMeters = meters;
  }

  public void setUpperLimit(double meters) {
    m_upperLimitMeters = meters;
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
  public void setControlMode(boolean isClosedLoop) {
    elevatorIsClosedLoop = isClosedLoop;
  }

  public boolean getControlMode() {
    return elevatorIsClosedLoop;
  }

  // Update elevator height using encoders and bottom limit switch
  public void updateElevatorHeight() {
    /* Uses limit switch to act as a baseline
     * to reset the sensor position and height to improve accuracy
     */
    if (getElevatorLowerSwitch()) {
      setElevatorSensorPosition(0.0);
    }
    elevatorHeight = getHeightMeters();
  }

  public Translation2d getElevatorTranslation() {
    return new Translation2d(
        getHeightMeters()
            * Math.cos(Constants.getInstance().Elevator.elevatorMountAngle.getRadians()),
        0);
  }

  public void updateShuffleboard() {
    // TODO: Add encoder counts per second or since last scheduler run

    elevatorHeightTab.setDouble(getHeightMeters());
    elevatorEncoderCountsTab.setDouble(getElevatorEncoderCounts());
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

  public void updateLog() {
    elevatorCurrentEntry.append(getElevatorMotorVoltage());
    elevatorSetpointEntry.append(desiredHeightValue);
    elevatorPositionEntry.append(elevatorHeight);
  }

  @Override
  public void simulationPeriodic() {
    elevatorSim.setInput(getElevatorPercentOutput() * 12);

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.020);

    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (elevatorSim.getPositionMeters()
                    / Constants.getInstance().Elevator.metersToEncoderCounts));

    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (elevatorSim.getVelocityMetersPerSecond()
                    / Constants.getInstance().Elevator.metersToEncoderCounts
                    * 10));

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    // This is why the mech2d is not proportional. We're using Units.metersToInches instead of
    // directly setting the length to meters
    // TODO: Make the mech2d proportional
    elevatorLigament2d.setLength(Units.metersToInches(elevatorSim.getPositionMeters()));
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    updateLog();
    // Yes, this needs to be called in the periodic. The simulation does not work without this
    updateShuffleboard();
    updateElevatorHeight();
    if (elevatorIsClosedLoop) {
      switch (desiredHeightState) {
        case JOYSTICK:
          desiredHeightValue = elevatorJoystickY * setpointMultiplier + getHeightMeters();
          break;
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
      // TODO: Cap the desiredHieghtValue by the min/max elevator height prior to setting it
      setElevatorMotionMagicMeters(desiredHeightValue);
    } else {
      // TODO: If targetElevatorLowerSwitch() is triggered, do not set a negative percent output
      setElevatorPercentOutput(elevatorJoystickY * setpointMultiplier);
    }
  }
}
