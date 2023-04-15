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

public class Intake extends SubsystemBase implements AutoCloseable {
  /** Creates a new Intake. */
  private boolean m_isIntaking = false;

  private INTAKE_STATE m_speed = INTAKE_STATE.NONE;

  private final TalonFX intakeMotor = new TalonFX(CAN.intakeMotor);

  private double previousVelocity;

  private double currentVelocity = intakeMotor.getSelectedSensorVelocity();

  //  private final DistanceSensor m_distanceSensor;

  // Log setup
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry currentEntry = new DoubleLogEntry(log, "/intake/current");

  // Mech2d setup
  private final MechanismLigament2d m_intakeLigament2d =
      new MechanismLigament2d("Intake", INTAKE.length, 0);

  public Intake() {
    //    m_distanceSensor = distanceSensor;
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
    //    return m_distanceSensor.getGamepieceDistanceInches(getHeldGamepiece());
    return 0;
  }

  // control mode function
  public void setIntaking(boolean isIntaking) {
    m_isIntaking = isIntaking;
  }

  public boolean isIntaking() {
    return m_isIntaking;
  }

  public void setIntakingState(INTAKE_STATE speed) {
    m_speed = speed;
  }

  public INTAKE_STATE getIntakeState() {
    return m_speed;
  }

  public double getMotorOutputCurrent() {
    return intakeMotor.getStatorCurrent();
  }

  public double getIntakeVelocity() {
    return intakeMotor.getSelectedSensorVelocity();
  }

  // set percent output function
  public void setPercentOutput(double value) {
    intakeMotor.set(ControlMode.PercentOutput, value);
  }

  private void updateIntakeState() {
    if (getIntakeState() == INTAKE_STATE.INTAKING_CONE) {
      if (getIntakeVelocity() < INTAKE.VELOCITY_THRESHOLDS.CONE_MIN.get())
        setIntakingState(INTAKE_STATE.HOLDING_CONE);
    }

    if (getIntakeState()
        == INTAKE_STATE.INTAKING_CUBE) { // Check if we are currently intaking a cube
      if (currentVelocity < INTAKE.VELOCITY_THRESHOLDS.CUBE_MIN.get())
        setIntakingState(INTAKE_STATE.HOLDING_CUBE);
    }

    setPercentOutput(getIntakeState().get());
  }

  // Shuffleboard or SmartDashboard function
  public void initSmartDashboard() {}

  public void updateSmartDashboard() {
    SmartDashboard.putString("Intake State", getIntakeState().toString());
  }

  public void updateLog() {
    currentEntry.append(getMotorOutputCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    updateLog();
    updateIntakeState();
    // TODO: If the cube or cone distance sensors see a game object, run the intake intakeMotor to
    // hold the game piece in.
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    m_intakeLigament2d.close();
  }
}
