// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.util.Units;

public class ConstantsAlpha extends ConstantsBeta {

  public static final class ElevatorDef {
    public static final TalonFXInvertType firstMotorInvert = TalonFXInvertType.CounterClockwise;
  }

  public static final class SwerveDriveDef {
    public static final double frontLeftCANCoderOffset = 327.129;
    public static final double frontRightCANCoderOffset = 62.051;
    public static final double backLeftCANCoderOffset = 11.250;
    public static final double backRightCANCoderOffset = 213.135;

    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;

    public static final double kP_Translation = 0.6;
    public static final double kI_Translation = 0;
    public static final double kD_Translation = 0;
    public static final double kP_Rotation = 4;
    public static final double kI_Rotation = 0;
    public static final double kD_Rotation = 0.01;
  }

  public static final class WristDef {
    public static final int wristMotor = 30;
    public static final int wristGearRatio = (1 / 50);
    public static final double encoderUnitsPerRotation = 0;
    private static final double wristRotationUpperLimit = 0;
    public static final double wristEncoderUpperLimit =
        wristRotationUpperLimit * encoderUnitsPerRotation * wristGearRatio;
    public static final double wristEncoderSlowdown = 0;
    public static final double maxSpeedLimitsPercent = 0;
    public static final int wristLowerSwitch = 0;
    public static final double wristmaxRotationDegrees = 0;
  }
}
