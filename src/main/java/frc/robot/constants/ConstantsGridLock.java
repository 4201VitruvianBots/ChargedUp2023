// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants.SwerveDriveModulePosition;
import frc.robot.utils.ModuleMap;
import java.util.Map;

public class ConstantsGridLock extends ConstantsRushHour {
  public static final class SwerveDriveDef {
    public static final double kTrackWidth = Units.inchesToMeters(24);
    public static final double kWheelBase = Units.inchesToMeters(24);

    public static final Map<SwerveDriveModulePosition, Translation2d> kModuleTranslations =
        Map.of(
            SwerveDriveModulePosition.FRONT_LEFT,
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            SwerveDriveModulePosition.FRONT_RIGHT,
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            SwerveDriveModulePosition.BACK_LEFT,
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            SwerveDriveModulePosition.BACK_RIGHT,
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double frontLeftCANCoderOffset = 105.557;
    public static final double frontRightCANCoderOffset = 67.412;
    public static final double backLeftCANCoderOffset = 96.240;
    public static final double backRightCANCoderOffset = 147.217;

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

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
