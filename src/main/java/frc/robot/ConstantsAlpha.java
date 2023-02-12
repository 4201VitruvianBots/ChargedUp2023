// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ModuleMap;
import frc.robot.Constants.SwerveDriveModulePosition;

public class ConstantsAlpha {

    public static final class OperatorConstantsDef {
        public static final int kDriverControllerPort = 0;
      }
    
      public static class USBDef {
        public static final int leftJoystick = 0;
        public static final int rightJoystick = 1;
        public static final int xBoxController = 2;
      }
    
      public final class PnuematicsDef {}
    
      public static final class ElevatorDef {
        public static final int elevatorMotorLeft = 32;
        public static final int elevatorMotorRight = 33;
    
        public static final int elevatorLowerSwitch = 8;
    
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
    
      public final class IntakeDef {
        public static final int intakeMotor = 31;
      }
    
      public final class WristDef {
        public static final int wristMotor = 30;
      }
    
      public final class LEDDef {
        public static final int CANdleID = 0;
      }
    
      public static final class CANDef { // TODO Not real number change tbt//
        public static final int pigeon = 9;
    
        public static final int frontLeftCanCoder = 10;
        public static final int frontRightCanCoder = 11;
        public static final int backLeftCanCoder = 12;
        public static final int backRightCanCoder = 13;
    
        public static final int frontLeftDriveMotor = 20;
        public static final int frontLeftTurnMotor = 21;
        public static final int frontRightDriveMotor = 22;
        public static final int frontRightTurnMotor = 23;
        public static final int backLeftDriveMotor = 24;
        public static final int backLeftTurnMotor = 25;
        public static final int backRightDriveMotor = 26;
        public static final int backRightTurnMotor = 27;
      }
    
      public static final class SwerveDriveDef {
        public static final double kTrackWidth = Units.inchesToMeters(24);
        public static final double kWheelBase = Units.inchesToMeters(24);
    
        public static final Map<SwerveDriveModulePosition, Translation2d> kModuleTranslations =
            Map.of(
                SwerveDriveModulePosition.FRONT_LEFT, new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                SwerveDriveModulePosition.FRONT_RIGHT, new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                SwerveDriveModulePosition.BACK_LEFT, new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                SwerveDriveModulePosition.BACK_RIGHT, new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
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
    
        public static final double kP_X = 0.6;
        public static final double kD_X = 0;
        public static final double kP_Y = 0.6;
        public static final double kD_Y = 0;
        public static final double kP_Theta = 4;
        public static final double kD_Theta = 0.01;

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
  
  public OperatorConstantsDef OperatorConstants;
  public USBDef USB;
  public PnuematicsDef Pnuematics;
  public ElevatorDef Elevator;
  public IntakeDef Intake;
  public WristDef Wrist;
  public LEDDef LED;
  public CANDef CAN;
  public SwerveDriveDef SwerveDrive;
  public SwerveModuleDef SwerveModule;

  //public ConstantsAlpha() {
    // final OperatorConstantsDef OperatorConstants = new OperatorConstantsDef();
    // final USBDef USB = new USBDef();
    // final PnuematicsDef Pnuematics = new PnuematicsDef();
    // final ElevatorDef Elevator = new ElevatorDef();
    // final IntakeDef Intake = new IntakeDef();
    // final WristDef Wrist = new WristDef();
    // final LEDDef LED = new LEDDef();
    // final CANDef CAN = new CANDef();
    // final SwerveDriveDef SwerveDrive = new SwerveDriveDef();
    // final SwerveModuleDef SwerveModule = new SwerveModuleDef();
  //}
}
