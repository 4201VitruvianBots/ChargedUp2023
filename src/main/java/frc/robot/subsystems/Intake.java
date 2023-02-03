// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static final String Controlmode = null;
  /** Creates a new Intake. */
  private boolean isIntaking = false;

  private TalonFX intakeMotor = new TalonFX(Constants.Intake.intakeMotor);
  private final PIDController intakePID = new PIDController(0.1, 0.01, 0.05);
  private double PercentOutput;

  public Intake() {
    // one or two motors

    // factory default cofigs
    intakeMotor.configFactoryDefault();
    intakePID.setTolerance(1);
    intakePID.enableContinuousInput(-180, 180);
    intakeMotor.setInverted(false);

    intakeMotor.setStatusFramePeriod(1, 255);
    intakeMotor.setStatusFramePeriod(2, 255);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configVoltageCompSaturation(10);
    intakeMotor.enableVoltageCompensation(true);
  }

  public void setIntakeSate(boolean state) {
    boolean isIntaking = state;
  }

  public void setSetpoint(double setpoint) {
    setpoint = PercentOutput;
    intakePID.setSetpoint(setpoint);
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
    intakeMotor.set(ControlMode.PercentOutput,value);
  }
  // shuffleboard or smartdashboard funciton
  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Intake", getIntakeState());
    SmartDashboard.putNumber("getIntake", 1);
    // PercentOutput = SmartDashboard.getNumber("IntakePercentOutput", PercentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // setIntakePercentOutput(intakePID.calculate(getMeasurement()));
  }
}
