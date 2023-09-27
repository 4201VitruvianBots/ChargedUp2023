// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.CONSTANTS.kCANCoderSensorUnitsPerRotation;
import static frc.robot.Constants.CONSTANTS.kFalconSensorUnitsPerRotation;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.utils.ModuleMap;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static String robotName = "";

  // Add any constants that do not change between robots here, as well as all
  // enums

  public static class USB {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;
  }

  public static final class CAN {
    public static final int CANdle = 0;
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

    public static final int wristMotor = 30;
    public static final int intakeMotor = 31;

    public static final int elevatorMotorLeft = 32;
    public static final int elevatorMotorRight = 33;
  }

  public static final class DIO {
    public static final int elevatorLowerLimitSwitch = 9;
    public static final int resetWristSwitch = 8; // TODO: Update when switch is put on robot
  }

  public static final class CONSTANTS {
    public static final int kFalconSensorUnitsPerRotation = 2048;
    public static final int kCANCoderSensorUnitsPerRotation = 4096;
  }

  public static final class ELEVATOR {
    // Elevator sim constants
    public static final DCMotor gearbox = DCMotor.getFalcon500(2);
    public static final double gearRatio = 8.82; // Real value 15.7?
    public static final double massKg = 4.0;
    public static final double drumRadiusMeters = Units.inchesToMeters(1.5);
    public static final Rotation2d mountAngleRadians = Rotation2d.fromDegrees(40);
    public static final double centerOffset = Units.inchesToMeters(14);
    public static final double carriageDistance = Units.inchesToMeters(7);
    public static final double carriageOffset = Units.inchesToMeters(11);
    public static final int mech2dAngleDegrees = 35;

    public static final double kMaxReverseOutput = -0.45;

    // PID
    public static final double kMaxVel = Units.inchesToMeters(238);
    public static final double kMaxAccel = Units.inchesToMeters(520);
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 0;

    public static final double encoderCountsToMeters =
        (drumRadiusMeters * 2 * Math.PI) / (kFalconSensorUnitsPerRotation * gearRatio);

    public static final double kG = 0.02;
    public static final double kV = 20.0; // 12.57;
    public static final double kA = 0.02; // 0.04;

    public static final double kP = 0.05;
    public static final double kI = 0.00;
    public static final double kD = 0.00;

    public static final double kPercentOutputMultiplier = 0.2;
    public static final double kLimitedPercentOutputMultiplier = 0.1;

    public static TalonFXInvertType mainMotorInversionType = TalonFXInvertType.Clockwise;

    public static final TrapezoidProfile.Constraints m_Constraints =
        new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel);

    public enum SETPOINT {
      STOWED(Units.inchesToMeters(0.0)),
      INTAKING_LOW(STOWED.get()),
      SCORE_LOW_REVERSE(Units.inchesToMeters(0.0)),
      SCORE_LOW_CONE(Units.inchesToMeters(4.0)),
      SCORE_LOW_CUBE(SCORE_LOW_CONE.get()),
      SCORE_MID_CONE(Units.inchesToMeters(24.0)),
      SCORE_MID_CUBE(Units.inchesToMeters(30.0)),
      SCORE_HIGH_CONE(Units.inchesToMeters(44.0)),
      SCORE_HIGH_CUBE(Units.inchesToMeters(45.0)),
      INTAKING_EXTENDED_CONE(Units.inchesToMeters(34.34)),
      INTAKING_EXTENDED_CUBE(Units.inchesToMeters(38.5));

      private final double value;

      SETPOINT(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }

    public enum THRESHOLD {
      // Units are in meters
      // Used to tell current zone for transitions
      ABSOLUTE_MIN(Units.inchesToMeters(0.0)),
      // switch to reset it
      ABSOLUTE_MAX(Units.inchesToMeters(50.0)),
      // NOTE: Zone limits should overlap to allow for transitions
      // Alpha 0 < x < 3.5 inches
      // Beta 3 < x < 28 inches
      // Gamma 27.5 < x < 50 inches
      ALPHA_MIN(ABSOLUTE_MIN.get()),
      ALPHA_MAX(Units.inchesToMeters(15.5)),
      BETA_MIN(Units.inchesToMeters(15.0)),
      BETA_MAX(Units.inchesToMeters(29)),
      GAMMA_MIN(Units.inchesToMeters(28.5)),
      GAMMA_MAX(ABSOLUTE_MAX.get());

      private final double value;

      THRESHOLD(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }
  }

  public static final class INTAKE {
    public static final double innerIntakeWidth = Units.inchesToMeters(15.5);
    public static final int leftConeSensorId = 1;
    public static final int rightConeSensorId = 2;
    public static final int cubeSensorId = 3;
    public static final double length = Units.inchesToMeters(12);

    public static final double gearRatio = 48.0 / 16.0;
    public static final DCMotor gearBox = DCMotor.getFalcon500(1);
    public static final double kMotorDistancePerPulse =
        360.0 / (kFalconSensorUnitsPerRotation * gearRatio);

    public static double kF = 0;
    public static double kP = 0.2;

    public enum THRESHOLDS {
      // Units are in raw motor velocity units
      NONE_MIN(9000),
      NONE_MAX(11000),
      CONE_MIN(8000),
      CONE_MAX(10000),
      CUBE_MIN(-6000),
      CUBE_MAX(-8000);

      private final double value;

      THRESHOLDS(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }

    public enum INTAKE_STATE {
      // Units are in Percent Output
      NONE(0),
      INTAKING_CONE(0.6),
      HOLDING_CONE(0.2),
      SCORING_CONE(-0.8),
      INTAKING_CUBE(-0.8),
      HOLDING_CUBE(-0.3),
      SCORING_CUBE(0.8);

      private final double value;

      INTAKE_STATE(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }

    public enum SENSOR_STATUS {
      UNREPORTED, // No status from the teensy is being reported
      DISCONNECTED, // The teensy failed to initialize the sensor
      TIMEOUT, // The sensor is connected, but failed to report a value in time
      FAILED, // The sensor reading is an obviously incorrect value (not between 0-8192)
      CONNECTED // The sensor is connected and is reading an expected value (between 0-8192)
    }
  }

  public static final class LED {

    public static final int LEDcount = 72;

    /** Different LED animation types */
    public enum ANIMATION_TYPE {
      ColorFlow,
      Fire,
      Larson,
      Rainbow,
      RgbFade,
      SingleFade,
      Strobe,
      Twinkle,
      TwinkleOff,
      Solid
    }
    // These color are channels passed in the setPattern() method in the LED subsystem
    public static final Color8Bit red = new Color8Bit(255, 0, 0);
    public static final Color8Bit green = new Color8Bit(0, 255, 0);
    public static final Color8Bit blue = new Color8Bit(0, 0, 255);
    public static final Color8Bit yellow = new Color8Bit(150, 120, 0);
    public static final Color8Bit purple = new Color8Bit(128, 0, 128);
    public static final Color8Bit orange = new Color8Bit(247, 116, 40);
    public static final Color8Bit pink = new Color8Bit(190, 30, 35);
    public static final Color8Bit white = new Color8Bit(125, 125, 125);
    public static final Color8Bit turquoise = new Color8Bit(24, 94, 89);
  }

  public static final class SWERVE_DRIVE {
    public static final double kTrackWidth = Units.inchesToMeters(24);
    public static final double kWheelBase = Units.inchesToMeters(24);

    public static final Map<SWERVE_MODULE_POSITION, Translation2d> kModuleTranslations =
        Map.of(
            SWERVE_MODULE_POSITION.FRONT_LEFT,
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            SWERVE_MODULE_POSITION.FRONT_RIGHT,
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            SWERVE_MODULE_POSITION.BACK_LEFT,
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            SWERVE_MODULE_POSITION.BACK_RIGHT,
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

            public static double frontLeftCANCoderOffset = 125.068;
            public static double frontRightCANCoderOffset = 62.051;
            public static double backLeftCANCoderOffset = 190.635;
            public static double backRightCANCoderOffset = 31.904;
            
    public static double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kLimitedSpeedMetersPerSecond = kMaxSpeedMetersPerSecond / 5;
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;
    public static final double kLimitedRotationRadiansPerSecond = kMaxRotationRadiansPerSecond / 5;

    public static final double kP_X = 0.6;
    public static final double kI_X = 0;
    public static final double kD_X = 0;
    public static final double kP_Y = 0.6;
    public static final double kI_Y = 0;
    public static final double kD_Y = 0;

    public static double kP_Theta = 4.0;
    public static double kI_Theta = 0;
    public static double kD_Theta = 0.01;

    public enum SWERVE_MODULE_POSITION {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }
  }

  public static class SWERVE_MODULE {
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurningMotorGearRatio = 150.0 / 7.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
    public static final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

    public static final double kDriveMotorDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (kFalconSensorUnitsPerRotation * kDriveMotorGearRatio);
    public static final double kTurningMotorDistancePerPulse =
        360.0 / (kFalconSensorUnitsPerRotation * kTurningMotorGearRatio);
    public static final double kTurnEncoderDistancePerPulse =
        360.0 / kCANCoderSensorUnitsPerRotation;

    public static final double ksDriveVoltSecondsPerMeter = 0.605 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 1.72 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.193 / 12;

    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  }

  public static final class VISION {
    public enum CAMERA_TYPE {
      OAK,
      LIMELIGHT,
      PHOTONVISION
    }

    public enum PIPELINE {
      DEFAULT(0),
      CUBE(1),
      CONE(2);

      private final int pipeline;

      PIPELINE(final int pipeline) {
        this.pipeline = pipeline;
      }

      public int get() {
        return pipeline;
      }
    }

    public enum CAMERA_SERVER {
      INTAKE("10.42.1.11"),
      LEFT_LOCALIZER("10.42.1.12"),
      RIGHT_LOCALIZER("10.42.1.13"),
      FUSED_LOCALIZER("10.42.1.12");

      private final String ip;

      CAMERA_SERVER(final String ip) {
        this.ip = ip;
      }

      @Override
      public String toString() {
        return ip;
      }
    }

    public static final Transform3d[] LOCALIZER_CAMERA_POSITION = {
      // Robot Center to Left Camera
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-(3 + (3.0 / 8.0))),
              Units.inchesToMeters(12),
              Units.inchesToMeters(20)),
          new Rotation3d()),
      // Robot Center to Right Camera
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-(3 + (3.0 / 8.0))),
              Units.inchesToMeters(-12),
              Units.inchesToMeters(20)),
          new Rotation3d()),
    };
  }

  public static final class WRIST {
    public static final double gearRatio = 1024.0 / 27.0;
    public static final double encoderUnitsToDegrees =
        360.0 / (kFalconSensorUnitsPerRotation * gearRatio);
    public static final DCMotor gearBox = DCMotor.getFalcon500(1);
    public static final double mass = Units.lbsToKilograms(20);
    public static final double length = Units.inchesToMeters(20);
    public static final double fourbarGearboxHeight = Units.inchesToMeters(4);
    public static final double fourbarAngleDegrees = 180;
    public static final int kTimeoutMs = 0;

    public static TalonFXInvertType motorInversionType = TalonFXInvertType.Clockwise;

    public static final double kPercentOutputMultiplier = 0.2;
    public static final double kLimitedPercentOutputMultiplier = 0.1;

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;

    // Values were experimentally determined
    // public static final double kMaxSlowVel = Units.degreesToRadians(400);
    // public static final double kMaxSlowAccel = Units.degreesToRadians(290);
    // public static final double kMaxFastVel = Units.degreesToRadians(400 * 1.25);
    // public static final double kMaxFastAccel = Units.degreesToRadians(290 * 1.25);

    public static final double kMaxVel = Units.degreesToRadians(720);
    public static final double kMaxAccel = Units.degreesToRadians(250);

    public static final TrapezoidProfile.Constraints m_constraints =
        new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel);

    public static final double FFkS = 0.06;
    public static final double kG = 0.54;
    public static final double FFkV = 1.6;
    public static final double kA = 0.02;

    public static final double kP = 0.04;
    public static final double kI = 0.0;
    public static final double kD = 1.0;

    public static final double kMaxPercentOutput = 1.0;
    public static final double kSetpointMultiplier = Units.degreesToRadians(60.0);

    public enum SETPOINT {
      // Units are in Radians
      STOWED(Units.degreesToRadians(98.0)),
      INTAKING_LOW_CUBE(Units.degreesToRadians(-13.5)),
      INTAKING_LOW_CONE(Units.degreesToRadians(16)),
      SCORE_LOW_REVERSE(Units.degreesToRadians(-14.0)),
      SCORE_LOW_CONE(Units.degreesToRadians(120.0)),
      SCORE_LOW_CUBE(SCORE_LOW_CONE.get()),
      SCORE_MID_CONE(Units.degreesToRadians(150.5)),
      SCORE_MID_CUBE(Units.degreesToRadians(132.0)),
      SCORE_HIGH_CONE(Units.degreesToRadians(151.5)),
      SCORE_HIGH_CUBE(Units.degreesToRadians(147.0)),
      INTAKING_EXTENDED_CONE(Units.degreesToRadians(121.3)),
      INTAKING_EXTENDED_CUBE(SCORE_HIGH_CUBE.get());

      private final double value;

      SETPOINT(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }

    public enum THRESHOLD {
      // Units are in radians
      ABSOLUTE_MIN(Units.degreesToRadians(-20.0)),
      ABSOLUTE_MAX(Units.degreesToRadians(180.0)),
      ALPHA_MIN(ABSOLUTE_MIN.get()),
      ALPHA_MAX(Units.degreesToRadians(110.0)),
      BETA_MIN(Units.degreesToRadians(25.0)),
      BETA_MAX(Units.degreesToRadians(150.0)),
      GAMMA_MIN(
          Units.degreesToRadians(
              40.0)), // TODO: Maybe change this to 25.0 like it was before as extended
      GAMMA_MAX(ABSOLUTE_MAX.get()),

      HORIZONTAL_LENGTH_MINUS15_CUBE(Units.inchesToMeters(17.0)),
      HORIZONTAL_LENGTH_MINUS15_CONE(Units.inchesToMeters(20.0)),
      HORIZONTAL_LENGTH_0_CUBE(Units.inchesToMeters(16.0)),
      HORIZONTAL_LENGTH_0_CONE(Units.inchesToMeters(19.5)),
      HORIZONTAL_LENGTH_90_CUBE(Units.inchesToMeters(-4.0)),
      HORIZONTAL_LENGTH_90_CONE(Units.inchesToMeters(-9.0)),
      HORIZONTAL_LENGTH_140_CUBE(Units.inchesToMeters(-17.0)),
      HORIZONTAL_LENGTH_140_CONE(Units.inchesToMeters(-25.0)),
      HORIZONTAL_LENGTH_180_CUBE(Units.inchesToMeters(-22.0)),
      HORIZONTAL_LENGTH_180_CONE(Units.inchesToMeters(-28.0));

      private final double value;

      THRESHOLD(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }
  }

  public static class STATE_HANDLER {

    public enum ZONE {
      UNDEFINED, // Danger
      STATUS, // Danger
      ALPHA,
      BETA,
      GAMMA,
    }

    public static final double elevatorSetpointTolerance = Units.inchesToMeters(2);
    public static final double wristSetpointTolerance = Units.degreesToRadians(4);

    public static final double universalWristLowerLimitRadians = Units.degreesToRadians(25.0);
    public static final double universalWristUpperLimitRadians = Units.degreesToRadians(115.0);

    public static boolean limitCanUtilization = false;

    public static final double mechanism2dXSize = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get() * 2;
    public static final double mechanism2dYSize = ELEVATOR.THRESHOLD.ABSOLUTE_MAX.get() * 2;
    public static final double mechanism2dXOffset = Units.inchesToMeters(3);
    public static final double mechanism2dYOffset = Units.inchesToMeters(11);

    public enum SUPERSTRUCTURE_STATE {
      // UNDEFINED
      DANGER_ZONE(ZONE.UNDEFINED),
      // STATUS
      DISABLED(ZONE.STATUS),
      ENABLED(ZONE.STATUS),
      LOW_BATTERY(ZONE.STATUS),
      // LOWs
      STOWED(ZONE.ALPHA),
      INTAKE_LOW_CONE(ZONE.ALPHA),
      INTAKE_LOW_CUBE(ZONE.ALPHA),
      SCORE_LOW_REVERSE(ZONE.ALPHA),
      SCORE_LOW(ZONE.ALPHA),
      SCORE_LOW_CONE(ZONE.ALPHA),
      SCORE_LOW_CUBE(ZONE.ALPHA),
      ALPHA_ZONE(ZONE.ALPHA),
      WRIST_IS_RESET(ZONE.ALPHA),
      // MID
      BETA_ZONE(ZONE.BETA),
      SCORE_MID(ZONE.BETA),
      SCORE_MID_CONE(ZONE.BETA),
      SCORE_MID_CUBE(ZONE.BETA),
      // HIGH
      GAMMA_ZONE(ZONE.GAMMA),
      INTAKE_EXTENDED(ZONE.GAMMA),
      SCORE_HIGH(ZONE.GAMMA),
      SCORE_HIGH_CONE(ZONE.GAMMA),
      SCORE_HIGH_CUBE(ZONE.GAMMA);

      // State Zone is determined by elevator setpoints
      private final ZONE zone;

      SUPERSTRUCTURE_STATE(final ZONE zone) {
        this.zone = zone;
      }

      public ZONE getZone() {
        return zone;
      }
    }

    public enum SETPOINT {
      // Units are in meters, radians
      STOWED(ELEVATOR.SETPOINT.STOWED.get(), WRIST.SETPOINT.STOWED.get()),
      SCORE_LOW(ELEVATOR.SETPOINT.SCORE_LOW_CONE.get(), WRIST.SETPOINT.SCORE_LOW_CONE.get()),
      SCORE_LOW_REVERSE(
          ELEVATOR.SETPOINT.SCORE_LOW_REVERSE.get(), WRIST.SETPOINT.SCORE_LOW_REVERSE.get()),
      SCORE_MID_CONE(ELEVATOR.SETPOINT.SCORE_MID_CONE.get(), WRIST.SETPOINT.SCORE_MID_CONE.get()),

      SCORE_MID_CUBE(ELEVATOR.SETPOINT.SCORE_MID_CUBE.get(), WRIST.SETPOINT.SCORE_MID_CUBE.get()),

      SCORE_HIGH_CONE(
          ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get(), WRIST.SETPOINT.SCORE_HIGH_CONE.get()),

      SCORE_HIGH_CUBE(
          ELEVATOR.SETPOINT.SCORE_HIGH_CUBE.get(), WRIST.SETPOINT.SCORE_HIGH_CUBE.get()),

      INTAKING_EXTENDED_CONE(
          ELEVATOR.SETPOINT.INTAKING_EXTENDED_CONE.get(),
          WRIST.SETPOINT.INTAKING_EXTENDED_CONE.get()),

      INTAKING_EXTENDED_CUBE(
          ELEVATOR.SETPOINT.INTAKING_EXTENDED_CUBE.get(),
          WRIST.SETPOINT.INTAKING_EXTENDED_CUBE.get()),

      INTAKING_LOW_CONE(
          ELEVATOR.SETPOINT.INTAKING_LOW.get(), WRIST.SETPOINT.INTAKING_LOW_CONE.get()),
      INTAKING_LOW_CUBE(
          ELEVATOR.SETPOINT.INTAKING_LOW.get(), WRIST.SETPOINT.INTAKING_LOW_CUBE.get());

      private final double elevatorSetpointMeters;
      private final double wristSetpointRadians;

      SETPOINT(double elevatorSetpointMeters, double wristSetpointRadians) {
        this.elevatorSetpointMeters = elevatorSetpointMeters;
        this.wristSetpointRadians = wristSetpointRadians;
      }

      public double getElevatorSetpointMeters() {
        return elevatorSetpointMeters;
      }

      public double getWristSetpointRadians() {
        return wristSetpointRadians;
      }
    }
  }

  public static class AUTO {
    public static double kAutoBalanceTimeout = 2.0;
    public static final double kAutoBalanceAngleThresholdDegrees = 2.0;

    public enum WAIT {
      SCORE_HIGH_CONE(0.65), // good
      SCORE_HIGH_CUBE(0.65), // good
      SCORE_MID_CONE(0.4),
      SCORE_MID_CUBE(0.4),

      WAIT_TO_PLACE_CONE(1), // good
      WAIT_TO_PLACE_CUBE(1),
      WAIT_TO_PLACE_CUBE_MID(0.7), // good

      SCORING_CONE(0.65), // good
      SCORING_CUBE(0.65), // good

      STOW_HIGH_CONE(1.1), // 1.1 // good
      STOW_HIGH_CUBE(1.1), // 1.1
      STOW_HIGH_CUBE_FAST(0.7333333333), // good

      STOW_MID_CONE(0.3),
      STOW_MID_CUBE(0.1),

      INTAKE_TO_STOW(0.5); // good

      private final double value;

      WAIT(double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }
  }

  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public static class UTIL {
    public static final String tempFileName = "initialize";
  }

  public enum SCORING_STATE {
    STOWED,
    AUTO_BALANCE,
    LOW_REVERSE,
    LOW,
    MID,
    MID_CONE,
    MID_CUBE,
    HIGH,
    HIGH_CONE,
    HIGH_CUBE,
    INTAKE,
    INTAKE_EXTENDED
  }

  private static void initBeta() {
    robotName = "Beta";
    SWERVE_DRIVE.frontLeftCANCoderOffset = 125.068; // 85.957;
    SWERVE_DRIVE.frontRightCANCoderOffset = 62.051; // 41.748;
    SWERVE_DRIVE.backLeftCANCoderOffset = 190.635; // 261.475;
    SWERVE_DRIVE.backRightCANCoderOffset = 31.113;
  }

  private static void initAlpha() {
    robotName = "Alpha";

    SWERVE_DRIVE.frontLeftCANCoderOffset = 126.914; // 85.957;
    SWERVE_DRIVE.frontRightCANCoderOffset = 222.9785; // 41.748;
    SWERVE_DRIVE.backLeftCANCoderOffset = 191.25; // 261.475;
    SWERVE_DRIVE.backRightCANCoderOffset = 34.7605;

    ELEVATOR.mainMotorInversionType = TalonFXInvertType.CounterClockwise;
    WRIST.motorInversionType = TalonFXInvertType.Clockwise;
  }

  private static void initSim() {
    robotName = "Sim";

    SWERVE_DRIVE.frontLeftCANCoderOffset = 0;
    SWERVE_DRIVE.frontRightCANCoderOffset = 0;
    SWERVE_DRIVE.backLeftCANCoderOffset = 0;
    SWERVE_DRIVE.backRightCANCoderOffset = 0;

    SWERVE_DRIVE.kP_Theta = 0.1;
    SWERVE_DRIVE.kI_Theta = 0;
    SWERVE_DRIVE.kD_Theta = 0;
  }

  private static void initUnknown() {
    robotName = "Unknown (Default Beta values)";
  }

  public static void initConstants() {
    String mac = "";
    try {
      var ip = InetAddress.getLocalHost();
      var networkInterfaces = NetworkInterface.getByInetAddress(ip).getHardwareAddress();
      String[] hex = new String[networkInterfaces.length];
      for (int i = 0; i < networkInterfaces.length; i++) {
        hex[i] = String.format("%02X", networkInterfaces[i]);
      }
      mac = String.join(":", hex);
    } catch (Exception e) {
      //      e.printStackTrace();
    }
    if (mac.equals(alphaRobotMAC)) {
      initAlpha();
    } else if (mac.equals(betaRobotMAC)) {
      initBeta();
    } else if (RobotBase.isSimulation()) {
      initSim();
    } else {
      initUnknown();
    }

    if (!DriverStation.isFMSAttached()) {
      STATE_HANDLER.limitCanUtilization = false;
    }

    SmartDashboard.putString("Robot Name", robotName);
  }

  public static final String alphaRobotMAC = "00:80:2F:25:BC:FD";
  public static final String betaRobotMAC = "00:80:2F:19:30:B7";
}
