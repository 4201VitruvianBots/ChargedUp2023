// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants.SwerveDriveModulePosition;
import frc.robot.utils.ModuleMap;
import java.util.Map;

public class ConstantsRushHour {

  public static final class ElevatorDef {
    // Elevator sim constants
    public static final DCMotor elevatorGearbox = DCMotor.getFalcon500(2);
    public static final double elevatorGearing = 10.0;
    public static final double elevatorMassKg = 4.0;
    public static final double elevatorDrumRadiusMeters = Units.inchesToMeters(1.0);
    public static final double elevatorMinHeightMeters = 0;
    public static final double elevatorMaxHeightMeters = Units.inchesToMeters(43.0);

    // PID
    public static final double kSensorUnitsPerRotation = 2048.0;
    public static final double kGearRatio = 1.0 / 5.0;
    public static final double kMaxRPM = 6380.0;
    public static final double kMaxVelocity =
        (kMaxRPM / 600) * (kSensorUnitsPerRotation / kGearRatio);

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 0;

    public static final double metersToEncoderCounts =
        (elevatorDrumRadiusMeters * 2 * Math.PI) / (kSensorUnitsPerRotation * kGearRatio);
  }

  public final class IntakeDef {}

  public final class WristDef {}

  public final class LEDDef {}

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

    public static final double frontLeftCANCoderOffset = 84.98628; // 85.957;
    public static final double frontRightCANCoderOffset = 219.506836; // 41.748;
    public static final double backLeftCANCoderOffset = 261.95906; // 261.475;
    public static final double backRightCANCoderOffset = 148.183594; // 148.008; // 329.150;

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

  public static final class SwerveModuleDef {
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurningMotorGearRatio = 150.0 / 7.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final int kFalconEncoderCPR = 2048;
    public static final int kCANCoderCPR = 4096;

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
    public static final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

    public static final double kDriveMotorDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (kFalconEncoderCPR * kDriveMotorGearRatio);
    public static final double kTurningMotorDistancePerPulse =
        360.0 / (kFalconEncoderCPR * kTurningMotorGearRatio);
    public static final double kTurningEncoderDistancePerPulse = 360.0 / kCANCoderCPR;

    public static final double ksDriveVoltSecondsPerMeter = 0.605 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 1.72 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.193 / 12;

    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  }

  public final ElevatorDef Elevator;
  public final IntakeDef Intake;
  public final WristDef Wrist;
  public final LEDDef LED;
  public final SwerveDriveDef SwerveDrive;
  public final SwerveModuleDef SwerveModule;

  public ConstantsRushHour() {
    Elevator = new ElevatorDef();
    Intake = new IntakeDef();
    Wrist = new WristDef();
    LED = new LEDDef();
    SwerveDrive = new SwerveDriveDef();
    SwerveModule = new SwerveModuleDef();
  }
}
