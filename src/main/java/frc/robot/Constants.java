// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.ModuleMap;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
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
  // Add any constants that do not change between robots here, as well as all enums

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
    public static final int elevatorLowerSwitch = 8;
  }

  public static final class ELEVATOR {
    public enum STATE {
      OPEN_LOOP_MANUAL,
      CLOSED_LOOP_MANUAL,
      SETPOINT
    }

    public enum SETPOINT {
      STOWED(Units.inchesToMeters(0.0)),
      INTAKING_LOW(STOWED.get()),
      SCORE_LOW_REVERSE(Units.inchesToMeters(0.0)),
      SCORE_LOW(Units.inchesToMeters(20.0)),
      SCORE_MID(Units.inchesToMeters(26.0)),
      SCORE_HIGH(Units.inchesToMeters(32.0)),
      INTAKING_EXTENDED(SCORE_HIGH.get());

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
      ABSOLUTE_MIN(Units.inchesToMeters(0.0)),
      ABSOLUTE_MAX(Units.inchesToMeters(50.0)),
      LOW_MIN(ABSOLUTE_MIN.get()),
      LOW_MAX(Units.inchesToMeters(16.0)),
      HIGH_MIN(Units.inchesToMeters(8.0)),
      HIGH_MAX(Units.inchesToMeters(24.0)),
      EXTENDED_MIN(Units.inchesToMeters(12.0)),
      EXTENDED_MAX(ABSOLUTE_MAX.get()),
      LOW_TO_HIGH(Units.inchesToMeters(14.0)),
      HIGH_TO_LOW(Units.inchesToMeters(10.0)),
      HIGH_TO_EXTENDED(Units.inchesToMeters(26.0)),
      EXTENDED_TO_HIGH(Units.inchesToMeters(22.0));

      private final double value;

      THRESHOLD(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }
  }

  public static final class SWERVEDRIVE {
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

    public enum SWERVE_MODULE_POSITION {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }
  }

  public static final class VISION {
    public enum CAMERA_TYPE {
      OAK,
      LIMELIGHT,
      PHOTONVISION
    }

    public enum CAMERA_SERVER {
      INTAKE("10.42.1.10"),
      OUTTAKE("10.42.1.10"),
      LEFT_LOCALIZER("10.42.1.11"),
      RIGHT_LOCALIZER("10.42.1.12"),
      FUSED_LOCALIZER("10.42.1.11");

      private final String ip;

      CAMERA_SERVER(final String ip) {
        this.ip = ip;
      }

      @Override
      public String toString() {
        return ip;
      }
    }

    public static Transform3d[] LOCALIZER_CAMERA_POSITION = {
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
    public enum STATE {
      OPEN_LOOP_MANUAL,
      CLOSED_LOOP_MANUAL,
      SETPOINT
    }

    public enum SETPOINT {
      // Units are in Radians
      STOWED(Units.degreesToRadians(90.0)),
      INTAKING_LOW(Units.degreesToRadians(-10.0)),
      SCORE_LOW_REVERSE(Units.degreesToRadians(-10.0)),
      SCORE_LOW(Units.degreesToRadians(180.0)),
      SCORE_MID(Units.degreesToRadians(180.0)),
      SCORE_HIGH(Units.degreesToRadians(180.0)),
      INTAKING_EXTENDED(SCORE_HIGH.get());

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
      ABSOLUTE_MIN(Units.degreesToRadians(-15.0)),
      ABSOLUTE_MAX(Units.degreesToRadians(225.0)),
      LOW_MIN(ABSOLUTE_MIN.get()),
      LOW_MAX(Units.degreesToRadians(130.0)),
      HIGH_MIN(Units.degreesToRadians(0.0)),
      HIGH_MAX(LOW_MAX.get()),
      EXTENDED_MIN(HIGH_MIN.get()),
      EXTENDED_MAX(ABSOLUTE_MAX.get());

      private final double value;

      THRESHOLD(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }
  }

  public static class STATEHANDLER {
    public enum WRIST_SETPOINT_OFFSET {
      LOW(Units.inchesToMeters(6)),
      MID(Units.inchesToMeters(6)),
      HIGH(Units.inchesToMeters(6));

      private final double value;

      WRIST_SETPOINT_OFFSET(final double value) {
        this.value = value;
      }

      public double get() {
        return value;
      }
    }
  }

  public enum SCORING_STATE {
    STOWED,
    AUTO_BALANCE,
    SETPOINT_LOW_INTAKE,
    SETPOINT_LOW,
    SETPOINT_MEDIUM,
    SETPOINT_HIGH,
    SMART_LOW_REVERSE,
    SMART_LOW,
    SMART_MEDIUM,
    SMART_HIGH,
  }

  public static class ConstantsBase {
    public String robotName;

    public static class ElevatorDefB {
      // Elevator sim constants
      public final DCMotor elevatorGearbox = DCMotor.getFalcon500(2);
      public final double elevatorGearing = 10.0;
      public final double elevatorMassKg = 4.0;
      public final double elevatorDrumRadiusMeters = Units.inchesToMeters(0.5625);
      public final Rotation2d elevatorMountAngle = Rotation2d.fromDegrees(40);

      // PID
      public final double kSensorUnitsPerRotation = 2048.0;
      public final double kGearRatio = 5.0;
      public final double kMaxRPM = 6380.0;
      public final double kMaxVelocity = (kMaxRPM / 600) * (kSensorUnitsPerRotation / kGearRatio);

      public final int kSlotIdx = 0;
      public final int kPIDLoopIdx = 0;
      public final int kTimeoutMs = 0;

      public final double encoderCountsToMeters =
          (elevatorDrumRadiusMeters * 2 * Math.PI) / (kSensorUnitsPerRotation * kGearRatio);

      public final double kS = 0.15;
      public final double kV = 12.57;
      public final double kA = 0.04;

      public final TalonFXInvertType mainMotorInversionType = TalonFXInvertType.Clockwise;
    }

    public static class IntakeDef {}

    public static class WristDefB {
      public final double wristGearRatio = 1024.0 / 27.0;
      public final double encoderUnitsToDegrees = 360.0 / (2048.0 * wristGearRatio);
      public final DCMotor gearBox = DCMotor.getFalcon500(1);
      public final double wristMass = Units.lbsToKilograms(20);
      public final double wristLength = Units.inchesToMeters(22);
      public final int wristLowerSwitch = 0;

      // Values were experimentally determined
      public final double kMaxVel = Units.degreesToRadians(360);
      public final double kMaxAccel = Units.degreesToRadians(360);
      public final double FFkS = 0.1;
      public final double FFkV = 1.95;
      public final double kG = 1.75;
      public final double kA = 0.16;

      public final double kP = 0.1;
      public final double kI = 0.0;
      public final double kD = 0.0;
    }

    public static class LEDDef {}

    public static class SwerveDriveDefB {
      public final double frontLeftCANCoderOffset = 16.26; // 85.957;
      public final double frontRightCANCoderOffset = 215.596; // 41.748;
      public final double backLeftCANCoderOffset = 262.705; // 261.475;
      public final double backRightCANCoderOffset = 151.348; // 148.008; // 329.150;

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

    public static class SwerveModuleDef {
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

    public ElevatorDefB Elevator;
    public IntakeDef Intake;
    public WristDefB Wrist;
    public LEDDef LED;
    public SwerveDriveDefB SwerveDrive;
    public SwerveModuleDef SwerveModule;

    public ConstantsBase() {
      Elevator = new ElevatorDefB();
      Intake = new IntakeDef();
      Wrist = new WristDefB();
      LED = new LEDDef();
      SwerveDrive = new SwerveDriveDefB();
      SwerveModule = new SwerveModuleDef();
      robotName = "";
    }
  }

  private static ConstantsBase m_constants;

  public static class ConstantsBeta extends ConstantsBase {
    public ConstantsBeta() {
      super();
      robotName = "Beta";
    }
  }

  public static class ConstantsAlpha extends ConstantsBeta {
    public static class ElevatorDefA extends ElevatorDefB {
      // Elevator sim constants
      public final DCMotor elevatorGearbox = DCMotor.getFalcon500(2);
      public final double elevatorGearing = 10.0;
      public final double elevatorMassKg = 4.0;
      public final double elevatorDrumRadiusMeters = Units.inchesToMeters(0.5625);
      public final Rotation2d elevatorMountAngle = Rotation2d.fromDegrees(40);

      // PID
      public final double kSensorUnitsPerRotation = 2048.0;
      public final double kGearRatio = 5.0;
      public final double kMaxRPM = 6380.0;
      public final double kMaxVelocity = (kMaxRPM / 600) * (kSensorUnitsPerRotation / kGearRatio);

      public final int kSlotIdx = 0;
      public final int kPIDLoopIdx = 0;
      public final int kTimeoutMs = 0;

      public final double encoderCountsToMeters =
          (elevatorDrumRadiusMeters * 2 * Math.PI) / (kSensorUnitsPerRotation * kGearRatio);

      public final double kS = 0.15;
      public final double kV = 12.57;
      public final double kA = 0.04;

      public final TalonFXInvertType mainMotorInversionType = TalonFXInvertType.CounterClockwise;
    }

    public static final class SwerveDriveDefA extends SwerveDriveDefB {
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

    public static final class WristDefA extends WristDefB {
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
      Elevator = new ElevatorDefA();
      SwerveDrive = new SwerveDriveDefA();
      Wrist = new WristDefA();
      robotName = "Alpha";
    }
  }

  public static ConstantsBase getInstance() {
    if (m_constants == null) {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      String mac = "";
      try {
        var ip = InetAddress.getLocalHost();
        var networkInterfaces = NetworkInterface.getByInetAddress(ip).getHardwareAddress();
        String[] hex = new String[networkInterfaces.length];
        for (int i = 0; i < networkInterfaces.length; i++) {
          hex[i] = String.format("%02X", networkInterfaces[i]);
        }
        mac = String.join(":", hex);
      } catch (SocketException | UnknownHostException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }

      if (mac.equals(alphaRobotMAC)) {
        m_constants = new ConstantsAlpha();
      } else {
        m_constants = new ConstantsBeta();
      }
    }
    SmartDashboard.putString("Robot Name", m_constants.robotName);
    return m_constants;
  }

  public static final String alphaRobotMAC = "00:80:2F:25:BC:FD";
  public static final String betaRobotMAC = "00:80:2F:19:30:B7";
}
