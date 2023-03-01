// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ConstantsAlpha extends ConstantsBeta {

  public static final class SwerveDriveDef {
    public static final double frontLeftCANCoderOffset = -197.402;
    public static final double frontRightCANCoderOffset = -36.211;
    public static final double backLeftCANCoderOffset = -82.002;
    public static final double backRightCANCoderOffset = -311.084;

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
    public static final double wristGearRatio = (1.0 / 50.0);
    public static final double encoderUnitsPerRotation = 0;
    private static final double wristRotationUpperLimit = 0;
    public static final double wristEncoderUpperLimit =
        wristRotationUpperLimit * encoderUnitsPerRotation * wristGearRatio;
    public static final double wristEncoderSlowdown = 0;
    public static final double maxSpeedLimitsPercent = 0;
    public static final int wristLowerSwitch = 0;
    public static final double wristmaxRotationDegrees = 0;
  }

  public ConstantsAlpha() {
    super();
    robotName = "Alpha";
  }
}
