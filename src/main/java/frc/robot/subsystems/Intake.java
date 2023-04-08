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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INTAKE;

public class Intake extends SubsystemBase implements AutoCloseable {
  /** Creates a new Intake. */
  private boolean isIntakingCone = false;

  private boolean isIntakingCube = false;

  private final TalonFX intakeMotor = new TalonFX(CAN.intakeMotor);
  private double m_percentOutput;

  // Log setup
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry currentEntry = new DoubleLogEntry(log, "/intake/current");

  // Mech2d setup
  public final MechanismLigament2d m_ligament2d =
      new MechanismLigament2d("Intake", INTAKE.length, 0);

  public Intake() {
    // one or two motors

    // factory default configs
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(true);

    intakeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    // set current limit on TalonFX motors
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 30, 0.1));
    intakeMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 30, 0.1));
    intakeMotor.setStatusFramePeriod(1, 255);
    intakeMotor.setStatusFramePeriod(2, 255);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configVoltageCompSaturation(10);
    intakeMotor.enableVoltageCompensation(true);

    intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    intakeMotor.config_kF(0, INTAKE.kF);
    intakeMotor.config_kP(0, INTAKE.kP);

    initSmartDashboard();
  }
  
  public MechanismLigament2d getLigament() {
    return m_ligament2d;
  }

  // TODO: Need two measurement values: One that averages the two used to measure the cone and
  // another to measure the
  //  distance to the cube
  public double getConeDistance() {
    return 0;
  }

  public double getCubeDistance() {
    return 0;
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

  // True if Cube is detected, otherwise assume Cone
  public INTAKE.INTAKE_STATE getHeldGamepiece() {
    if (getConeDistance() > Units.inchesToMeters(15)
        && getCubeDistance() > Units.inchesToMeters(15)) return INTAKE.INTAKE_STATE.NONE;
    else if (getConeDistance() < Units.inchesToMeters(13)) return INTAKE.INTAKE_STATE.CONE;
    else if (getCubeDistance() < Units.inchesToMeters(14)) return INTAKE.INTAKE_STATE.CUBE;

    return INTAKE.INTAKE_STATE.NONE;
  }

  public double getMotorOutputCurrent() {
    return intakeMotor.getStatorCurrent();
  }

  // set percent output function
  public void setPercentOutput(double value) {
    intakeMotor.set(ControlMode.PercentOutput, value);
  }

  // Shuffleboard or SmartDashboard function
  public void initSmartDashboard() {
    m_ligament2d.setColor(new Color8Bit(255, 114, 118)); // Light red
  }

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
    // hold the game piece in.
    if (!isIntakingCone && !isIntakingCube) {
      if (getConeDistance() > 0) {
        m_percentOutput = 0;
      } else if (getCubeDistance() > 0) {
        m_percentOutput = 0;
      } else {
        m_percentOutput = 0;
      }
      setPercentOutput(m_percentOutput);
    }
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
