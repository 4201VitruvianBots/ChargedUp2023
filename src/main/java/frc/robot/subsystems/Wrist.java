// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Wrist extends SubsystemBase {
  private static final double maxRotationValue = Constants.constants.Wrist.wristmaxRotationDegrees;
  private static final int encoderCountsPerAngle = 0;
  private static final boolean wristLowerLimitOverride = false;
  private static double desiredRotationValue;
  private static WristRotations desiredRotationState = WristRotations.NONE;
  private double wristPosition = 0;
  private static double wristJoystickX;
  private boolean wristIsClosedLoop = true; 

  public enum WristRotations {
    LOW,
    MEDIUM,
    JOYSTICK,
    NONE
  }

  private static DigitalInput wristLowerSwitch =
      new DigitalInput(Constants.constants.Wrist.wristLowerSwitch);
  /** Creates a new Wrist. */
  private static TalonFX wristMotor = new TalonFX(Constants.constants.Wrist.wristMotor);

  private static final double kDt = 0.02;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(Constants.constants.Wrist.kV, Constants.constants.Wrist.kA);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // Create a new ArmFeedforward with gains kS, kG, kV, and kA
  private ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.constants.Wrist.FFkS,
          Constants.constants.Wrist.kG,
          Constants.constants.Wrist.FFkV,
          Constants.constants.Wrist.kA);

  private boolean isWristMoving;
  private final double kF = 0;
  private final double kP = 0.2;
  private double setpointMultiplier = 1;

  // Logging setup

  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry wristCurrentEntry = new DoubleLogEntry(log, "/wrist/wristCurrent");
  public DoubleLogEntry wristSetpointEntry = new DoubleLogEntry(log, "/wrist/wristSetpoint");
  public DoubleLogEntry wristPositionEntry = new DoubleLogEntry(log, "/wrist/wristPosition");
  public static ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");
  public GenericEntry wristHeightTab = wristTab.add("Wrist Position", 0).getEntry();
  public GenericEntry wristDesiredTab = wristTab.add("Wrist DesiredRotation", 0).getEntry();

  public Wrist() {
    // One motor for the wrist

    // factory default configs
    wristMotor.configFactoryDefault();

    wristMotor.setInverted(true);

    wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    wristMotor.setStatusFramePeriod(1, 0);
    wristMotor.setStatusFramePeriod(2, 0);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configVoltageCompSaturation(10);
    wristMotor.enableVoltageCompensation(true);

    wristMotor.config_kF(0, kF);
    wristMotor.config_kP(0, kP);
  }
  //  setpoint for the wrist
  public void setSetpoint(double setpoint) {
    this.desiredRotationValue = setpoint;
    this.m_goal = new TrapezoidProfile.State(desiredRotationValue, 0);
  }

  public boolean getWristState() {
    return isWristMoving;
  }

  public void setWristState(boolean state) {
    isWristMoving = state;
  }

  public double getDegrees(int motorIndex) {
    return wristMotor.getSelectedSensorVelocity()
        * (360.0 / Constants.constants.Wrist.encoderUnitsPerRotation)
        / Constants.constants.Wrist.wristGearRatio;
  }
  // this is get current angle
  public double getAngle() {
    return getWristPosition() / encoderCountsPerAngle;
  }

  public void zeroEncoder() {
    if (getLimitSwitchState(0)) {
      wristMotor.setSelectedSensorPosition(kP, 0, 0);
      isWristMoving = true;
    } else if (getLimitSwitchState(1)) {
      wristMotor.setSelectedSensorPosition(kF, 0, 0);
      isWristMoving = true;
    } else isWristMoving = false;
  }

  private boolean getLimitSwitchState(int i) {
    return false;
  }

  public void setEncoderPosition(int position) {
    wristMotor.setSelectedSensorPosition(position, 0, 0);
  }
  // reset angle of the wrist
  // set sensor to 0
  public void ResetWrist() {
    setSetpoint(desiredRotationValue); // setWristSensorPosition(0);
  }
  // code to limit the minimum/maximum setpoint of the wrist/ might be status frames
  public double getWristMotorVoltage() {
    return wristMotor.getMotorOutputVoltage();
  }

  // set percent output function
  // period function that edits the elevators height, from there make sure it obeys the limit (27.7
  // rotation)
  public void setWristPercentOutput(double value) {
    wristMotor.set(ControlMode.PercentOutput, value);
  }

  private double getWristPosition() {
    return wristMotor.getSelectedSensorPosition() / Constants.constants.Wrist.wristGearRatio;
  }
  // reset wrist angle function based off of a limit switch/hall effect sensor
  public static void updateWristRotation() {
    if (getWristLowerSwitch()) {
      setWristSensorPosition(0.0);
    }
  }

  public static void setWristSensorPosition(double position) {
    wristMotor.setSelectedSensorPosition(position);
  }

  public static boolean getWristLowerSwitch() {
    return !wristLowerSwitch.get();
  }

  public void setWristClosedLoop(boolean isClosedLoop) {
    wristIsClosedLoop = isClosedLoop;
  }

  public boolean getElevatorControlLoop() {
    return wristIsClosedLoop;
  }


  // smartdashboard function
  public void updateSmartDashboard() {
    // ShuffleboardTab.putNumber("Wrist", getWristState());
    SmartDashboard.putNumber("getWrist", 1);
    SmartDashboard.putNumber("Wrist Position", getWristPosition());
    SmartDashboard.putNumber("Elevator desiredrotationvalue", desiredRotationValue);
  }

  public void updateLog() {
    wristCurrentEntry.append(getWristMotorVoltage());
    // wristSetpointEntry.append();
    wristPositionEntry.append(getWristPosition());
  }

  @Override
  public void periodic() {

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.calculate(kDt);

    wristHeightTab.setDouble(getWristPosition());
    wristDesiredTab.setDouble(desiredRotationValue);

    wristMotor.set(
        TalonFXControlMode.Position,
        m_feedforward.calculate(Math.toRadians(getWristPosition()), m_setpoint.velocity));

    // This method will be called once per scheduler run
    if (wristIsClosedLoop) {
    switch (desiredRotationState) {
      case JOYSTICK:
      desiredRotationValue = wristJoystickX * setpointMultiplier + getWristPosition();
      case LOW:
        setSetpoint(0.0);
        break;
      case MEDIUM:
        setSetpoint(maxRotationValue / 2); // Placeholder values
        break;
      case NONE:
        break;
    }
  }
  else {
    setWristPercentOutput(wristJoystickX * setpointMultiplier);
  }
  }

  public Object getWristDesiredRotationState() {
    return desiredRotationValue;
  }

  public static void setWristJoystickX(
      double m_JoystickX) { // change name to call what its used for
    wristJoystickX = m_JoystickX;
  }

  public Object getWristDesiredRotations() {
    return desiredRotationState;
  }

  public void setWristDesiredRotationState(WristRotations wristEnum) {
    desiredRotationState = wristEnum;
  }
}
