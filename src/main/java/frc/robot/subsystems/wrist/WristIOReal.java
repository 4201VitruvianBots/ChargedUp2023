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

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CAN;
import frc.robot.Constants.WRIST;

/** Add your docs here. */
public class WristIOReal implements WristIO {
    protected final TalonFX wristMotor = new TalonFX(CAN.wristMotor);

    public WristIOReal() {
    // Factory default configs
    wristMotor.configFactoryDefault();
    wristMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    //Configure PID
    wristMotor.config_kP(0, WRIST.kP);
    wristMotor.config_kI(0, WRIST.kI);
    wristMotor.config_kD(0, WRIST.kD);
    wristMotor.configPeakOutputForward(WRIST.kMaxPercentOutput, WRIST.kTimeoutMs);
    wristMotor.configPeakOutputReverse(-WRIST.kMaxPercentOutput, WRIST.kTimeoutMs);
    wristMotor.setInverted(WRIST.motorInversionType);

    // TODO: Review limits, test to see what is appropriate or not
    wristMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 30, 0.2));

    wristMotor.configAllowableClosedloopError(0, 1 / WRIST.encoderUnitsToDegrees);

    
  }

    public void updateInputs(WristIOInputs inputs) {
      inputs.percentOutput = wristMotor.getMotorOutputPercent();
      inputs.outputVoltage = wristMotor.getMotorOutputVoltage();
      inputs.outputCurrent = wristMotor.getStatorCurrent();
      inputs.simEncoderSign = wristMotor.getInverted() ? -1 : 1;
      inputs.positionRadians = Units.degreesToRadians(wristMotor.getSelectedSensorPosition() * WRIST.encoderUnitsToDegrees);
      inputs.velocityDegreesPerSecond = wristMotor.getSelectedSensorVelocity() * WRIST.encoderUnitsToDegrees * 10;
    }

    public void setPercentOutput(double output) {
        wristMotor.set(ControlMode.PercentOutput, output);
      }

    // set percent output function with a boolean to enforce limits
  public void setPercentOutput(double output, boolean enforceLimits) {}

  public void setIValue(double value) {
    wristMotor.config_kI(0, value);
  }

    // reset angle of the wrist. ~-15 degrees is the position of the wrist when the intake is touching
  // the ground.
  public void resetAngleDegrees(double angleDegrees) {
    wristMotor.setSelectedSensorPosition(angleDegrees / WRIST.encoderUnitsToDegrees);
  }
  

  // Sets the setpoint of the wrist using a state calculated in periodic
  public void setSetpointTrapezoidState(TrapezoidProfile.State state, double pos) {
    wristMotor.set(
        ControlMode.Position,
        Units.radiansToDegrees(state.position) / WRIST.encoderUnitsToDegrees,
        DemandType.ArbitraryFeedForward,
        pos);
  }
  
  public void setPIDvalues(double f, double p, double i, double d, double izone) {
    wristMotor.config_kF(WRIST.kSlotIdx, f);
    wristMotor.config_kP(WRIST.kSlotIdx, p);
    wristMotor.config_kI(WRIST.kSlotIdx, i);
    wristMotor.config_kD(WRIST.kSlotIdx, d);
    wristMotor.config_IntegralZone(WRIST.kSlotIdx, izone);
  }
}