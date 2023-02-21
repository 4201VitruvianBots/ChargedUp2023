// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static boolean isIntaking = false;

  private final double kF = 0;
  private final double kP = 0.2;
  private TalonFX intakeMotor = new TalonFX(Constants.CAN.intakeMotor);
  private double m_percentOutput;

  // Log setup
  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry intakeCurrentEntry = new DoubleLogEntry(log, "/intake/intakeCurrent");

  public Intake() {
    // one or two motors

    // factory default configs
    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);

    intakeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    intakeMotor.setStatusFramePeriod(1, 255);
    intakeMotor.setStatusFramePeriod(2, 255);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configVoltageCompSaturation(10);
    intakeMotor.enableVoltageCompensation(true);

    intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    intakeMotor.config_kF(0, kF);
    intakeMotor.config_kP(0, kP);
  }

  // TODO: Need two measurement values: One that averages the two used to measure the cone and
  // another to measure the
  //  distance to the cube
  public double getIntakeConeMeasurement() {
    return 0;
  }

  public double getIntakeCubeMeasurement() {
    return 0;
  }

  // control mode function
  public boolean getIntakeState() {
    return isIntaking;
  }

  public double getIntakeMotorCurrent() {
    return intakeMotor.getStatorCurrent();
  }

  public void setIntakeState(boolean state) {
    isIntaking = state;
  }

  // set percent output function
  public void setIntakePercentOutput(double value) {
    intakeMotor.set(ControlMode.PercentOutput, value);
  }
  // Shuffleboard or SmartDashboard function
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Intake", getIntakeState());
  }

  public void updateLog() {
    intakeCurrentEntry.append(getIntakeMotorCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateLog();
    // TODO: If the cube or cone distance sensors see a game object, run the intake motor to hold
    // the game piece in.
    if (!isIntaking) {
      if (getIntakeConeMeasurement() > 0) {
        m_percentOutput = 0;
      } else if (getIntakeCubeMeasurement() > 0) {
        m_percentOutput = 0;
      } else {
        m_percentOutput = 0;
      }
      setIntakePercentOutput(m_percentOutput);
    }
  }
}
