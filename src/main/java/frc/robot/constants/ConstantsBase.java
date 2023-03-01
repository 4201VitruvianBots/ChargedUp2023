// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ConstantsBase {

  public final class ElevatorDef {
    // Elevator sim constants
    public final DCMotor elevatorGearbox = DCMotor.getFalcon500(2);
    public final double elevatorGearing = 10.0;
    public final double elevatorMassKg = 4.0;
    public final double elevatorDrumRadiusMeters = Units.inchesToMeters(0.5625);
    public final double elevatorMinHeightMeters = 0;
    public final double elevatorMaxHeightMeters = Units.inchesToMeters(43.0);
    // TODO: Find these two values
    public final double elevatorHeightWristUpperLimit = Units.inchesToMeters(12.0);
    public final double elevatorHeightWristLowerLimit = Units.inchesToMeters(12.0);
    public final Rotation2d elevatorMountAngle = Rotation2d.fromDegrees(0);

    // PID
    public final double kSensorUnitsPerRotation = 2048.0;
    public final double kGearRatio = 5.0;
    public final double kMaxRPM = 6380.0;
    public final double kMaxVelocity = (kMaxRPM / 600) * (kSensorUnitsPerRotation / kGearRatio);

    public final int kSlotIdx = 0;
    public final int kPIDLoopIdx = 0;
    public final int kTimeoutMs = 0;

    public final double metersToEncoderCounts =
        (elevatorDrumRadiusMeters * 2 * Math.PI) / (kSensorUnitsPerRotation * kGearRatio);

    public final double kS = 0.15;
    public final double kV = 12.57;
    public final double kA = 0.04;

    public final TalonFXInvertType firstMotorInvert = TalonFXInvertType.CounterClockwise;
  }

  public final class IntakeDef {}

  public static final class WristDef {
    public final double wristGearRatio = 1024.0 / 27.0;
    public final double encoderUnitsPerRotation = 360.0 / (2048.0 * wristGearRatio);
    public final DCMotor gearBox = DCMotor.getFalcon500(1);
    public final double wristMass = Units.lbsToKilograms(15);
    public final double wristLength = Units.inchesToMeters(22);
    public final double wristAbsoluteUpperLimitDegrees = 200.0;
    // TODO: Determine the soft limits
    public final double wristSoftUpperLimitDegrees = 130.0;
    public final double wristSoftLowerLimitDegrees = 0.0;
    public final double wristAbsoluteLowerLimitDegrees = -15.0;
    public final int wristLowerSwitch = 0;

    // Values were experimentally determined
    public final double FFkS = 0.1;
    public final double FFkV = 1.95;
    public final double kG = 1.75;
    public final double kA = 0.16;

    public final double kP = 0.04;
    public final double kD = 0.0;
  }

  public final class LEDDef {}

  public final class SwerveDriveDef {
    public final double frontLeftCANCoderOffset = 345.059; // 85.957;
    public final double frontRightCANCoderOffset = 56.909; // 41.748;
    public final double backLeftCANCoderOffset = 11.25; // 261.475;
    public final double backRightCANCoderOffset = 212.6515; // 148.008; // 329.150;

    public final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;

    public final double kP_Translation = 0.6;
    public final double kI_Translation = 0;
    public final double kD_Translation = 0;
    public final double kP_Rotation = 4;
    public final double kI_Rotation = 0;
    public final double kD_Rotation = 0.01;
  }

  public final class SwerveModuleDef {
    public final double kDriveMotorGearRatio = 6.12;
    public final double kTurningMotorGearRatio = 150.0 / 7.0;
    public final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public final int kFalconEncoderCPR = 2048;
    public final int kCANCoderCPR = 4096;

    public final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
    public final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

    public final double kDriveMotorDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (kFalconEncoderCPR * kDriveMotorGearRatio);
    public final double kTurningMotorDistancePerPulse =
        360.0 / (kFalconEncoderCPR * kTurningMotorGearRatio);
    public final double kTurningEncoderDistancePerPulse = 360.0 / kCANCoderCPR;

    public final double ksDriveVoltSecondsPerMeter = 0.605 / 12;
    public final double kvDriveVoltSecondsSquaredPerMeter = 1.72 / 12;
    public final double kaDriveVoltSecondsSquaredPerMeter = 0.193 / 12;

    public final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  }

  public static final TalonFXInvertType firstMotorInvert = null;

  public final ElevatorDef Elevator;
  public final IntakeDef Intake;
  public final WristDef Wrist;
  public final LEDDef LED;
  public final SwerveDriveDef SwerveDrive;
  public final SwerveModuleDef SwerveModule;

  public ConstantsBase() {
    Elevator = new ElevatorDef();
    Intake = new IntakeDef();
    Wrist = new WristDef();
    LED = new LEDDef();
    SwerveDrive = new SwerveDriveDef();
    SwerveModule = new SwerveModuleDef();
  }
}
