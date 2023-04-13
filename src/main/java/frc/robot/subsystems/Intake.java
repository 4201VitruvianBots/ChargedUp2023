// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.utils.DistanceSensor;

public class Intake extends SubsystemBase implements AutoCloseable {
  /** Creates a new Intake. */
  private boolean isIntakingCone = false;

  private INTAKE_STATE m_intakeMode= INTAKE_STATE.CONE; 

  private boolean isIntakingCube = false;

  private final TalonFX intakeMotor = new TalonFX(CAN.intakeMotor);
  private double m_percentOutput;

  private final DistanceSensor m_distanceSensor;

  // Log setup
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry currentEntry = new DoubleLogEntry(log, "/intake/current");

  // Mech2d setup
  private final MechanismLigament2d m_intakeLigament2d =
      new MechanismLigament2d("Intake", INTAKE.length, 0);

  public Intake(DistanceSensor distanceSensor) {
    m_distanceSensor = distanceSensor;
    // one or two motors

    // factory default configs
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(true);

    intakeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    // set current limit on TalonFX motors
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 30, 0.1));
    intakeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 30, 0.1));
    intakeMotor.setStatusFramePeriod(1, 255);
    intakeMotor.setStatusFramePeriod(2, 255);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configVoltageCompSaturation(10);
    intakeMotor.enableVoltageCompensation(true);

    intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    intakeMotor.config_kF(0, INTAKE.kF);
    intakeMotor.config_kP(0, INTAKE.kP);

    initSmartDashboard();

    m_intakeLigament2d.setColor(new Color8Bit(255, 114, 118)); // Light red
  }

  public INTAKE.INTAKE_STATE getHeldGamepiece() {
    double leftConeSensorValue =
        m_distanceSensor.getSensorValueMillimeters(INTAKE.leftConeSensorId) / 1000.0;
    double rightConeSensorValue =
        m_distanceSensor.getSensorValueMillimeters(INTAKE.rightConeSensorId) / 1000.0;
    double cubeSensorValue =
        m_distanceSensor.getSensorValueMillimeters(INTAKE.cubeSensorId) / 1000.0;

    if (leftConeSensorValue + rightConeSensorValue <= INTAKE.innerIntakeWidth) {
      return INTAKE.INTAKE_STATE.CONE;
    } else if (cubeSensorValue <= INTAKE.innerIntakeWidth - 1) {
      return INTAKE.INTAKE_STATE.CUBE;
    } else {
      return INTAKE.INTAKE_STATE.NONE;
    }
  }

  // Returns a pose where the center of the gamepiece should be
  public Pose2d getGamepiecePose(Pose2d intakePose) {
    return new Pose2d(
        intakePose.getX(),
        intakePose.getY() + getGamepieceDistanceInches(),
        intakePose.getRotation());
  }

  public MechanismLigament2d getLigament() {
    return m_intakeLigament2d;
  }

  public double getGamepieceDistanceInches() {
    return m_distanceSensor.getGamepieceDistanceInches(getHeldGamepiece());
  }

  // control mode function
  public boolean getIntakeState() {
    return isIntakingCone || isIntakingCube;
  }

  public boolean getIntakeConeState() {
    return isIntakingCone;
  }

  public void setIntakeStateCone(boolean state) {
    isIntakingCone = state;
  }

  public boolean getIntakeCubeState() {
    return isIntakingCube;
  }

  public void setIntakeStateCube(boolean state) {
    isIntakingCube = state;
  }

  public double getMotorOutputCurrent() {
    return intakeMotor.getStatorCurrent();
  }

  // set percent output function
  public void setPercentOutput(double value) {
    intakeMotor.set(ControlMode.PercentOutput, value);
  }

  public void setIntakeMode(INTAKE_STATE mode) {
    m_intakeMode = mode;
  }

  public INTAKE_STATE getIntakeMode() {
    return m_intakeMode; 
  }

  // Shuffleboard or SmartDashboard function
  public void initSmartDashboard() {}

  public void updateSmartDashboard() {
    // TODO: Consolidate this using the INTAKE_STATE enum
    SmartDashboard.putBoolean("Intaking Cone", getIntakeConeState());
    SmartDashboard.putBoolean("Intaking Cube", getIntakeConeState());
  }

  public void updateLog() {
    currentEntry.append(getMotorOutputCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    updateLog();
    // TODO: If the cube or cone distance sensors see a game object, run the intake intakeMotor to
    // hold
    // the game piece in.
    if (!getIntakeState()) {
      if (m_distanceSensor.getConeDistanceInches() > 0) {
        m_percentOutput = 0;
      } else if (m_distanceSensor.getCubeDistanceInches() > 0) {
        m_percentOutput = 0;
      } else {
        m_percentOutput = 0;
      }
      setPercentOutput(m_percentOutput);
    }
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    if (m_intakeLigament2d != null) m_intakeLigament2d.close();
  }
}
