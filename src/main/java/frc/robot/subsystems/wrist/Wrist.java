// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CONTROL_MODE;
import frc.robot.Constants.DIO;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.WRIST;
import frc.robot.Constants.WRIST.THRESHOLD;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.wrist.WristIO.WristIOInputs;

public class Wrist extends SubsystemBase implements AutoCloseable {

  private boolean m_wristInitialized = false;

  private final WristIO m_io;
  private final WristIOInputsAutoLogged m_inputs = new WristIOInputsAutoLogged();
  
  private final DigitalInput resetSwitch = new DigitalInput(DIO.resetWristSwitch);

  private double m_desiredSetpointRadians;
  private double m_lowerLimitRadians = THRESHOLD.ABSOLUTE_MIN.get();
  private double m_upperLimitRadians = THRESHOLD.ABSOLUTE_MAX.get();

  private CONTROL_MODE m_controlMode = CONTROL_MODE.CLOSED_LOOP;
  private boolean m_testMode = false;

  private double m_joystickInput;
  private boolean m_limitJoystickInput;
  private boolean m_userSetpoint;

  private final Intake m_intake;

  private TrapezoidProfile.Constraints m_currentConstraints = WRIST.m_constraints;

  private Translation2d m_wristHorizontalTranslation = new Translation2d();

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // Create a new ArmFeedforward with gains kS, kG, kV, and kA
  private final ArmFeedforward m_feedForward =
      new ArmFeedforward(WRIST.FFkS, WRIST.kG, WRIST.FFkV, WRIST.kA);
  private ArmFeedforward m_currentFeedForward = m_feedForward;

  // This timer is used in trapezoid profile to calculate the amount of time since the last periodic
  // run
  private final Timer m_timer = new Timer();
  private double m_lastTimestamp = 0;

  private double m_currentKI = 0;
  private double m_newKI = 0;

  // Simulation setup
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          WRIST.gearBox,
          WRIST.gearRatio,
          SingleJointedArmSim.estimateMOI(WRIST.length, WRIST.mass),
          WRIST.length,
          THRESHOLD.ABSOLUTE_MIN.get(),
          THRESHOLD.ABSOLUTE_MAX.get(),
          false
          // VecBuilder.fill(2.0 * Math.PI / 2048.0) // Add noise with a std-dev of 1 tick
          );
  private static int m_simEncoderSign = 1;

  // Mech2d setup
  private final MechanismLigament2d m_wristGearboxLigament2d =
      new MechanismLigament2d("FourbarGearbox", WRIST.fourbarGearboxHeight, 90);
  private final MechanismLigament2d m_wristLigament2d =
      new MechanismLigament2d("Fourbar", WRIST.length, WRIST.fourbarAngleDegrees);

  // Logging setup
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry voltageEntry = new DoubleLogEntry(log, "/wrist/voltage");
  private final DoubleLogEntry currentEntry = new DoubleLogEntry(log, "/wrist/current");
  private final DoubleLogEntry desiredPositionEntry =
      new DoubleLogEntry(log, "/wrist/desiredPositionDegrees");
  private final DoubleLogEntry commandedPositionEntry =
      new DoubleLogEntry(log, "/wrist/commandedPositionDegrees");
  private final DoubleLogEntry positionDegreesEntry =
      new DoubleLogEntry(log, "/wrist/positionDegrees");

  private DoublePublisher kCommandedAngleDegreesPub;
  private DoublePublisher kDesiredAngleDegreesPub;
  private DoublePublisher kCurrentAngleDegreesPub;
  private DoublePublisher currentTrapezoidVelocity;
  private DoublePublisher currentTrapezoidAcceleration;
  private StringPublisher currentCommandStatePub;

  

  // Workaround for wrist not setting angle properly/inversion race condition
  private void initializeWristAngle() {
    if (DriverStation.isDisabled() && !m_wristInitialized) {
      double wristResetAngleDegrees = -15.0;
      m_io.resetAngleDegrees(wristResetAngleDegrees);

      if (Math.abs(getPositionDegrees() - wristResetAngleDegrees) <= 0.05)
        m_wristInitialized = true;
    }
  }

  public void setWristInitialized(boolean state) {
    m_wristInitialized = state;
  }

  public MechanismLigament2d getGearboxLigament() {
    return m_wristGearboxLigament2d;
  }

  public MechanismLigament2d getWristLigament() {
    return m_wristLigament2d;
  }

  public void setUserInput(double input) {
    m_joystickInput = input;
  }

  public void setJoystickLimit(boolean limit) {
    m_limitJoystickInput = limit;
  }

  public void setUserSetpoint(boolean bool) {
    m_userSetpoint = bool;
  }

  public boolean isUserControlled() {
    return m_joystickInput != 0 && !m_userSetpoint;
  }

  public void setClosedLoopControlMode(CONTROL_MODE mode) {
    m_controlMode = mode;
  }

  public CONTROL_MODE getClosedLoopControlMode() {
    return m_controlMode;
  }

  public boolean isClosedLoopControl() {
    return m_controlMode == CONTROL_MODE.CLOSED_LOOP;
  }

  public void setPercentOutput(double output) {
    setPercentOutput(output, false);
  }

  // set percent output function with a boolean to enforce limits
  private void setPercentOutput(double output, boolean enforceLimits) {
    if (enforceLimits) {
      if (getPositionRadians() > (getUpperLimit() - Units.degreesToRadians(1))) {
        output = Math.min(output, 0);
      }
      if (getPositionRadians() < (getLowerLimit() + Units.degreesToRadians(0.1))) {
        output = Math.max(output, 0);
      }
    }

    m_io.setPercentOutput(output);
  }






  private double calculateFeedforward(TrapezoidProfile.State state) {
    return (m_currentFeedForward.calculate(state.position, state.velocity) / 12.0);
  }

  public void setTestMode(boolean mode) {
    m_testMode = mode;
  }


  public void setArmMotorFeedForward(double s, double g, double v, double a) {
    m_currentFeedForward = new ArmFeedforward(s, g, v, a);
  }

  public void setTrapezoidalConstraints(double maxVel, double maxAccel) {
    m_currentConstraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
  }

  // Sets the setpoint of the wrist to its current position to keep it in place
  public void resetTrapezoidState() {
    m_setpoint =
        new TrapezoidProfile.State(
            getPositionRadians(), Units.degreesToRadians(m_inputs.velocityDegreesPerSecond));
  }

  public void setSetpointPositionRadians(double desiredAngleRadians) {
    m_desiredSetpointRadians = desiredAngleRadians;
  }

  public double getDesiredPositionRadians() {
    return m_desiredSetpointRadians;
  }

  public double getPositionRadians() {
    return Units.degreesToRadians(getPositionDegrees());
  }

  public double getPositionDegrees() {
    return Units.radiansToDegrees(m_inputs.positionRadians);
  }
  
  // Converts the angle of the wrist into a Rotation2d object to be applied to a Pose2d
  public Rotation2d getWristAngleRotation2d() {
    return Rotation2d.fromDegrees(getPositionDegrees());
  }

  public void setLowerLimit(double radians) {
    m_lowerLimitRadians = radians;
  }

  public double getLowerLimit() {
    return m_lowerLimitRadians;
  }

  public void setUpperLimit(double radians) {
    m_upperLimitRadians = radians;
  }

  public double getUpperLimit() {
    return m_upperLimitRadians;
  }

  // TODO: Recalculate values using new intake/elevator
  public void updateHorizontalTranslation() {
    // Cube: f(x)=0.00000874723*t^3-0.00218403*t^2-0.101395*t+16;
    // Cone: f(x)=0.000860801*t^2-0.406027*t+16.3458;
    // Cube: f(x)=0.00000913468*t^3-0.00232508*t^2-0.0894341*t+16.1239;
    // Cone: f(x)=-0.270347*t+16.8574;
    double horizontalDistance = 0;
    if (m_intake.getIntakeState() == INTAKE_STATE.HOLDING_CONE)
      horizontalDistance =
          0.00000913468 * Math.pow(getPositionDegrees(), 3)
              - 0.00232508 * Math.pow(getPositionDegrees(), 2)
              - 0.0894341 * getPositionDegrees()
              + 16.1239;
    else if (m_intake.getIntakeState() == INTAKE_STATE.HOLDING_CUBE)
      horizontalDistance =
          //          0.00860801 * Math.pow(getPositionDegrees(), 2) +
          -0.270347 * getPositionDegrees() + 16.8574;
    m_wristHorizontalTranslation = new Translation2d(Units.inchesToMeters(horizontalDistance), 0);
  }

  public Translation2d getHorizontalTranslation() {
    return m_wristHorizontalTranslation;
  }

  private void updateIValue() {
    if (getPositionDegrees() < 30) m_newKI = 0.00001;
    else if (getPositionDegrees() >= 30) m_newKI = 0;

    if (m_currentKI != m_newKI) {
      m_io.setIValue(m_newKI);
      m_currentKI = m_newKI;
    }
  }

  public void setSetpointTrapezoidState(TrapezoidProfile.State state) {
    m_io.setSetpointTrapezoidState(state, calculateFeedforward(state));
  }

  public void setPIDValues(double f, double p, double i, double d, double iZone) {
    m_io.setPIDValues(f, p, i, d, iZone);
  }

  public void resetAngleDegrees(double angleDegrees) {
    m_io.resetAngleDegrees(angleDegrees);
  }

  private void initSmartDashboard() {
    SmartDashboard.putData(this);

    NetworkTable wristTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Wrist");

    kCommandedAngleDegreesPub = wristTab.getDoubleTopic("Commanded Angle Degrees").publish();
    kDesiredAngleDegreesPub = wristTab.getDoubleTopic("Desired Angle Degrees").publish();
    kCurrentAngleDegreesPub = wristTab.getDoubleTopic("Current Angle Degrees").publish();
    currentCommandStatePub = wristTab.getStringTopic("Command State").publish();
    currentTrapezoidAcceleration = wristTab.getDoubleTopic("Trapezoid Acceleration").publish();
    currentTrapezoidVelocity = wristTab.getDoubleTopic("Trapezoid Velocity").publish();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putString("Wrist Closed Loop", getClosedLoopControlMode().name());
    SmartDashboard.putNumber("Wrist Angles Degrees", getPositionDegrees());

    currentCommandStatePub.set(getClosedLoopControlMode().toString());
    kDesiredAngleDegreesPub.set(Units.radiansToDegrees(getDesiredPositionRadians()));
    kCurrentAngleDegreesPub.set(getPositionDegrees());
  }

  // public void updateLog() {
  //   voltageEntry.append(getMotorOutputVoltage());
  //   currentEntry.append(getMotorOutputCurrent());
  //   desiredPositionEntry.append(getDesiredPositionRadians());
  //   positionDegreesEntry.append(getPositionDegrees());
  // }

  @Override
  public void periodic() {
    initializeWristAngle();

    if (!m_testMode) {
      updateIValue();
    }

    updateSmartDashboard();
    //    updateLog();

    TrapezoidProfile profile;
    double currentTime = m_timer.get();
    switch (m_controlMode) {
      case OPEN_LOOP:
        double percentOutput = m_joystickInput * WRIST.kPercentOutputMultiplier;

        // Limit the percent output of the wrist joystick when the stick is pressed down to make
        // small adjustments
        if (m_limitJoystickInput)
          percentOutput = m_joystickInput * WRIST.kLimitedPercentOutputMultiplier;

        setPercentOutput(percentOutput, true);
        break;
      case CLOSED_LOOP:
      default:
        m_goal = new TrapezoidProfile.State(m_desiredSetpointRadians, 0);
        profile = new TrapezoidProfile(m_currentConstraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(currentTime - m_lastTimestamp);
        m_lastTimestamp = currentTime;

        setSetpointTrapezoidState(m_setpoint);
        break;
    }
  }


  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    m_wristLigament2d.close();
    resetSwitch.close();
  }
}
