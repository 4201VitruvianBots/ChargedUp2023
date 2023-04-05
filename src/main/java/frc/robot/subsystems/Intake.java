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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.INTAKE;
import frc.robot.utils.DistanceSensor;

public class Intake extends SubsystemBase implements AutoCloseable {
  /** Creates a new Intake. */
  private static boolean isIntakingCone = false;

  private static boolean isIntakingCube = false;

  private final TalonFX intakeMotor = new TalonFX(CAN.intakeMotor);
  private double m_percentOutput;

  private final DistanceSensor m_distanceSensor;

  // Log setup
  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry currentEntry = new DoubleLogEntry(log, "/intake/current");

  public Intake(DistanceSensor distanceSensor) {
    m_distanceSensor = distanceSensor;

    // one or two motors

    // factory default configs
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(true);

    intakeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    // set current limit on talonfx motors
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
  }

  // control mode function
  public boolean getIntakeState() {
    return isIntaking;
  }

  public boolean getIntakeStateCone() {
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
  // Shuffleboard or SmartDashboard function

  public void updateSmartDashboard() {
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
    if (!isIntaking) {
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

  @Override
  public void close() throws Exception {}
}
