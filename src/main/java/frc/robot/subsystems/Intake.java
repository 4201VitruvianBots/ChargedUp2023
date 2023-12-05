// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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

  private boolean m_useCubeSetpoint = false;
  private boolean m_retractIntake = false;

  private INTAKE_STATE m_state = INTAKE_STATE.NONE;

  private final TalonFX intakeMotor = new TalonFX(CAN.intakeMotor);

  //  private final DistanceSensor m_distanceSensor;

  // Log setup
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry currentEntry = new DoubleLogEntry(log, "/intake/current");

  private final FlywheelSim m_intakeSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.8, 0.6), INTAKE.gearBox, INTAKE.gearRatio);
  private double m_simDistance;

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
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 40, 0.1));
    intakeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 50, 0.1));
    intakeMotor.setStatusFramePeriod(1, 255);
    intakeMotor.setStatusFramePeriod(2, 255);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configVoltageCompSaturation(10);
    intakeMotor.enableVoltageCompensation(true);

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
    m_state = speed;
  }

  public INTAKE_STATE getIntakeState() {
    return m_state;
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

  public double getSupplyCurrent() {
    return intakeMotor.getSupplyCurrent();
  }

  public TalonFX getIntakeMotor() {
    return intakeMotor;
  }

  private void updateIntakeState() {
    //    if (getIntakeState() == INTAKE_STATE.INTAKING_CONE) {
    //      if (getIntakeVelocity() < THRESHOLDS.CONE_MIN.get()) {
    //        setIntakingState(INTAKE_STATE.HOLDING_CONE);
    //        m_retractIntake = true;
    //      }
    //    }
    //
    //    if (getIntakeState()
    //        == INTAKE_STATE.INTAKING_CUBE) { // Check if we are currently intaking a cube
    //      if (getIntakeVelocity() < THRESHOLDS.CUBE_MIN.get()) {
    //        setIntakingState(INTAKE_STATE.HOLDING_CUBE);
    //        m_retractIntake = true;
    //      }
    //    }
    setPercentOutput(getIntakeState().get());
  }

  public boolean getRetractIntake() {
    return m_retractIntake;
  }

  public void setRetractIntake(boolean value) {
    m_retractIntake = value;
  }

  public boolean isUsingCubeSetpoint() {
    return m_useCubeSetpoint;
  }

  public void setUsingCubeSetpoint(boolean state) {
    m_useCubeSetpoint = state;
  }

  // Shuffleboard or SmartDashboard function
  public void initSmartDashboard() {}

  public void updateSmartDashboard() {
    SmartDashboard.putString("Intake State", getIntakeState().toString());
    SmartDashboard.putNumber("Intake Velocity", getIntakeVelocity());
    SmartDashboard.putNumber("Intake Percent Output", intakeMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("Intake Bus Voltage", intakeMotor.getBusVoltage());
    SmartDashboard.putNumber("Intake Error Derivative", intakeMotor.getErrorDerivative());
    SmartDashboard.putNumber("Intake Intergal Accumulator", intakeMotor.getIntegralAccumulator());
    SmartDashboard.putNumber("Intake Motor Output Voltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("Intake Sensor Velocity", intakeMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Intake Stator Current", intakeMotor.getStatorCurrent());
    SmartDashboard.putNumber("Intake Supply Current", intakeMotor.getSupplyCurrent());
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
    if (getSupplyCurrent() >= 4 && getSupplyCurrent() <= 5) {
      setIntakingState(INTAKE_STATE.HOLDING_CONE);

    } else if (m_state == INTAKE_STATE.SCORING_CONE
        || m_state == INTAKE_STATE.SCORING_CUBE
        || m_state == INTAKE_STATE.INTAKING_CUBE
        || m_state == INTAKE_STATE.INTAKING_CONE) {
      setIntakingState(m_state);
    }
  }

  @Override
  public void simulationPeriodic() {
    m_intakeSim.setInputVoltage(MathUtil.clamp(intakeMotor.getMotorOutputVoltage(), -12, 12));

    double dt = StateHandler.getSimDt();
    m_intakeSim.update(dt);
    m_simDistance += m_intakeSim.getAngularVelocityRadPerSec() * dt;

    Unmanaged.feedEnable(20);

    intakeMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition((int) (m_simDistance / INTAKE.kMotorDistancePerPulse));
    intakeMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_intakeSim.getAngularVelocityRadPerSec() / (INTAKE.kMotorDistancePerPulse * 10)));

    intakeMotor.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    m_intakeLigament2d.close();
  }
}
