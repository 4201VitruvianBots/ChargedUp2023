// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public static final int wristLowerSwitch = 0;
  }

  public static final class ELEVATOR {
    // Elevator sim constants
    public static final DCMotor gearbox = DCMotor.getFalcon500(2);
    public static final double gearRatio = 15.7;
    public static final double massKg = 4.0;
    public static final double drumRadiusMeters = Units.inchesToMeters(1.5);
    public static final Rotation2d mountAngleRadians = Rotation2d.fromDegrees(40);
    public static final double centerOffset = Units.inchesToMeters(10);

    // PID
    public static final double kSensorUnitsPerRotation = 2048.0;
    public static double kMaxVel = Units.inchesToMeters(25);
    public static double kMaxAccel = Units.inchesToMeters(20);
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 0;

    public static final double encoderCountsToMeters =
        (drumRadiusMeters * 2 * Math.PI) / (kSensorUnitsPerRotation * gearRatio);

    public static final double kS = 0.15956;
    public static final double kV = 8.4689; //12.57;
    public static final double kA = 0.23382; //0.04;

    public static TalonFXInvertType mainMotorInversionType = TalonFXInvertType.CounterClockwise;

    public enum STATE {
      OPEN_LOOP_MANUAL,
      CLOSED_LOOP_MANUAL,
      USER_SETPOINT,
      AUTO_SETPOINT
    }

    public enum SETPOINT {
      STOWED(Units.inchesToMeters(0.0)),
      INTAKING_LOW(STOWED.get()),
      SCORE_LOW_REVERSE(Units.inchesToMeters(0.0)),
      SCORE_LOW_CONE(Units.inchesToMeters(4.0)),
      SCORE_LOW_CUBE(SCORE_LOW_CONE.get()),
      SCORE_MID_CONE(Units.inchesToMeters(21)),
      SCORE_MID_CUBE(SCORE_MID_CONE.get()),
      SCORE_HIGH_CONE(Units.inchesToMeters(40)),
      SCORE_HIGH_CUBE(SCORE_HIGH_CONE.get()),
      INTAKING_EXTENDED(Units.inchesToMeters(19.13));

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
      // NOTE: Zone limits should overlap to allow for transitions
      LOW_MIN(ABSOLUTE_MIN.get()),
      LOW_MAX(Units.inchesToMeters(11)),
      MID_MIN(Units.inchesToMeters(9)),
      MID_MAX(Units.inchesToMeters(14)),
      HIGH_MIN(Units.inchesToMeters(12)),
      HIGH_MAX(Units.inchesToMeters(18)),
      EXTENDED_MIN(Units.inchesToMeters(16)),
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

  public static final class INTAKE {
    public static final double innerIntakeWidth = Units.inchesToMeters(15.5);
    public static final int leftConeSensorId = 0;
    public static final int rightConeSensorId = 1;
    public static final int cubeSensorId = 2;

    public enum HELD_GAMEPIECE {
      NONE,
      CUBE,
      CONE
    }
  }

  public static final class LED {}

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

    public static double frontLeftCANCoderOffset =  125.8985;
    public static double frontRightCANCoderOffset = 229.351;
    public static double backLeftCANCoderOffset =  190.591;
    public static double backRightCANCoderOffset = 31.6845;
    
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;

    public static final double kP_Translation = 0.6;
    public static final double kI_Translation = 0;
    public static final double kD_Translation = 0;
    public static final double kP_Rotation = 4;
    public static final double kI_Rotation = 0;
    public static final double kD_Rotation = 0.01;

    public enum SWERVE_MODULE_POSITION {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }
  }

  public static class SWERVEMODULE {
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurningMotorGearRatio = 150.0 / 7.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final int kFalconEncoderCPR = 2048;
    public final int kCANCoderCPR = 4096;

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
    public static final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

    public static final double kDriveMotorDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (kFalconEncoderCPR * kDriveMotorGearRatio);
    public static final double kTurningMotorDistancePerPulse =
        360.0 / (kFalconEncoderCPR * kTurningMotorGearRatio);
    public final double kTurnEncoderDistancePerPulse = 360.0 / kCANCoderCPR;

    public static final double ksDriveVoltSecondsPerMeter = 0.605 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 1.72 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.193 / 12;

    public final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
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
    public static final double gearRatio = 1024.0 / 27.0;
    public static final double encoderUnitsToDegrees = 360.0 / (2048.0 * gearRatio);
    public static final DCMotor gearBox = DCMotor.getFalcon500(1);
    public static final double mass = Units.lbsToKilograms(20);
    public static final double length = Units.inchesToMeters(22);
    public static int kTimeoutMs = 0;

    public static TalonFXInvertType motorInversionType = TalonFXInvertType.Clockwise;

    // Values were experimentally determined
    public static final double kMaxSlowVel = Units.degreesToRadians(400);
    public static final double kMaxSlowAccel = Units.degreesToRadians(290);
    public static final double kMaxFastVel = Units.degreesToRadians(360 * 1.35);
    public static final double kMaxFastAccel = Units.degreesToRadians(250 * 1.35);
    public static final double FFkS = 0.1;
    public static final double kG = 1.75;
    public static final double FFkV = 1.95;
    public static final double kA = 0.16;

    public static final double kP = 0.15;
    public static double kI = 0.0001;
    public static final double kD = 0.0;

    public enum STATE {
      OPEN_LOOP_MANUAL,
      CLOSED_LOOP_MANUAL,
      USER_SETPOINT,
      AUTO_SETPOINT
    }

    public enum SETPOINT {
      // Units are in Radians
      STOWED(Units.degreesToRadians(90.0)),
      INTAKING_LOW(Units.degreesToRadians(-13.0)),
      SCORE_LOW_REVERSE(Units.degreesToRadians(-10.0)),
      SCORE_LOW_CONE(Units.degreesToRadians(120.0)),
      SCORE_LOW_CUBE(SCORE_LOW_CONE.get()),
      SCORE_MID_CONE(Units.degreesToRadians(143.0)),
      SCORE_MID_CUBE(SCORE_MID_CONE.get()),
      SCORE_HIGH_CONE(Units.degreesToRadians(146)),
      SCORE_HIGH_CUBE(SCORE_HIGH_CONE.get()),
      INTAKING_EXTENDED(SCORE_HIGH_CONE.get());

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
      ABSOLUTE_MAX(Units.degreesToRadians(180.0)),
      LOW_MIN(ABSOLUTE_MIN.get()),
      LOW_MAX(Units.degreesToRadians(100.0)),
      MID_MIN(Units.degreesToRadians(10.0)),
      MID_MAX(Units.degreesToRadians(110.0)),
      HIGH_MIN(MID_MIN.get()),
      HIGH_MAX(Units.degreesToRadians(120.0)),
      EXTENDED_MIN(HIGH_MIN.get()),
      EXTENDED_MAX(ABSOLUTE_MAX.get()),
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

  public static class STATEHANDLER {}

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

  private static void initBeta() {
    robotName = "Beta";
    // SWERVEDRIVE.frontLeftCANCoderOffset = 81.431; // 85.957;
    // SWERVEDRIVE.frontRightCANCoderOffset = 219.4625; // 41.748;
    // SWERVEDRIVE.backLeftCANCoderOffset = 191.382; // 261.475;
    // SWERVEDRIVE.backRightCANCoderOffset = 32.6515;
    SWERVEDRIVE.frontLeftCANCoderOffset = 125.8985; // 85.957;
    SWERVEDRIVE.frontRightCANCoderOffset = 229.351; // 41.748;
    SWERVEDRIVE.backLeftCANCoderOffset = 190.591; // 261.475;
    SWERVEDRIVE.backRightCANCoderOffset = 31.6845;
  }

  private static void initAlpha() {
    robotName = "Alpha";

    SWERVEDRIVE.frontLeftCANCoderOffset = 126.914; // 85.957;
    SWERVEDRIVE.frontRightCANCoderOffset = 222.9785; // 41.748;
    SWERVEDRIVE.backLeftCANCoderOffset = 191.25; // 261.475;
    SWERVEDRIVE.backRightCANCoderOffset = 34.7605;

    ELEVATOR.mainMotorInversionType = TalonFXInvertType.CounterClockwise;
    WRIST.motorInversionType = TalonFXInvertType.Clockwise;
  }

  private static void initSim() {
    robotName = "Sim";

    SWERVEDRIVE.frontLeftCANCoderOffset = 0;
    SWERVEDRIVE.frontRightCANCoderOffset = 0;
    SWERVEDRIVE.backLeftCANCoderOffset = 0;
    SWERVEDRIVE.backRightCANCoderOffset = 0;
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
      // TODO Auto-generated catch block
      e.printStackTrace();
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
    SmartDashboard.putString("Robot Name", robotName);
  }

  public static final String alphaRobotMAC = "00:80:2F:25:BC:FD";
  public static final String betaRobotMAC = "00:80:2F:19:30:B7";
}
