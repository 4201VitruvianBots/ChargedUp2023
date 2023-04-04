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
import frc.robot.Constants;
import frc.robot.Constants.INTAKE;

public class Intake extends SubsystemBase implements AutoCloseable {
  /** Creates a new Intake. */
  private static boolean isIntaking = false;

  private static boolean isIntakingCone = false;
  private static boolean isIntakingCube = false;

  private final double kF = 0;
  private final double kP = 0.2;
  private final TalonFX intakeMotor = new TalonFX(Constants.CAN.intakeMotor);
  private double m_percentOutput;

  // Log setup
  public DataLog log = DataLogManager.getLog();
  public DoubleLogEntry currentEntry = new DoubleLogEntry(log, "/intake/current");

  public Intake() {
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
    intakeMotor.config_kF(0, kF);
    intakeMotor.config_kP(0, kP);
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

  public boolean getIntakeStateCube() {
    return isIntakingCube;
  }

  public void setIntakeStateCube(boolean state) {
    isIntakingCube = state;
  }

  // True if Cube is detected, otherwise assume Cone
  public INTAKE.HELD_GAMEPIECE getHeldGamepiece() {
    if (getConeDistance() > Units.inchesToMeters(15)
        && getCubeDistance() > Units.inchesToMeters(15)) return INTAKE.HELD_GAMEPIECE.NONE;
    else if (getConeDistance() < Units.inchesToMeters(13)) return INTAKE.HELD_GAMEPIECE.CONE;
    else if (getCubeDistance() < Units.inchesToMeters(14)) return INTAKE.HELD_GAMEPIECE.CUBE;

    return INTAKE.HELD_GAMEPIECE.NONE;
  }

  public double getMotorOutputCurrent() {
    return intakeMotor.getStatorCurrent();
  }

  public void setBooleanState(boolean state) {
    isIntaking = state;
  }

  // set percent output function
  public void setPercentOutput(double value) {
    intakeMotor.set(ControlMode.PercentOutput, value);
  }
  // Shuffleboard or SmartDashboard function

  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Intake", getIntakeState());
    SmartDashboard.putNumber("IntakeMotorSpeed", intakeMotor.getMotorOutputPercent());
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

  @Override
  public void close() throws Exception {}
}
