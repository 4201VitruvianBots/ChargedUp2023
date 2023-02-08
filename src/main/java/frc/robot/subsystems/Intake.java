// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static boolean isIntaking = false;

  private final double kF = 0;
  private final double kP = 0.2;
  private TalonFX intakeMotor = new TalonFX(Constants.Intake.intakeMotor);
  private double PercentOutput;

  public Intake() {
    // one or two motors

    // factory default cofigs
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

  public double getMeasurement() {
    return intakeMotor.getSelectedSensorPosition();
  }

  // control mode function
  public static boolean getIntakeState() {
    return isIntaking;
  }

  public void setIntakeState(boolean b) {
    isIntaking = b;
  }
  // set percent output function
  public void setIntakePercentOutput(double value) {
    intakeMotor.set(ControlMode.PercentOutput, value);
  }
  // shuffleboard or smartdashboard funciton
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Intake", getIntakeState());
    SmartDashboard.putNumber("getIntake", 1);
    PercentOutput = SmartDashboard.getNumber("IntakePercentOutput", PercentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
