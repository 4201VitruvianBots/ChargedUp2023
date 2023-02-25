// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.ModuleMap;
import java.util.Map;
import java.util.Objects;

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
      LOW(Units.inchesToMeters(20.0)),
      MID(Units.inchesToMeters(26.0)),
      HIGH(Units.inchesToMeters(32.0)),
      INTAKING_EXTENDED(HIGH.get());

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
      ABSOLUTE_MAX(Units.inchesToMeters(43.0)),
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

  public static final class SwerveDrive {
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

    public enum CAMERA_LOCATION {
      INTAKE,
      OUTTAKE,
      LEFT_LOCALIZER,
      RIGHT_LOCALIZER
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

    public enum SERVER_IPS {
      INTAKE("10.42.1.10"),
      LEFT_LOCALIZER("10.42.1.11"),
      RIGHT_LOCALIZER("10.42.1.12");

      private final String ip;

      SERVER_IPS(final String ip) {
        this.ip = ip;
      }

      @Override
      public String toString() {
        return ip;
      }
    }
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
      LOW(Units.degreesToRadians(180.0)),
      MID(Units.degreesToRadians(180.0)),
      HIGH(Units.degreesToRadians(180.0)),
      INTAKING_EXTENDED(HIGH.get());

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
    SMART_LOW_INTAKE,
    SMART_LOW,
    SMART_MEDIUM,
    SMART_HIGH,
  }

  private static ConstantsBase m_constants;

  public static ConstantsBase getInstance() {
    if (m_constants == null) {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      String mac = inst.getTable("RIO-Info").getEntry("MAC").getString("N/A");
      if (Objects.equals(mac, Constants.alphaRobotMAC)) {
        m_constants = new ConstantsRushHour();
      } else {
        m_constants = new ConstantsGridLock();
      }
    }
    return m_constants;
  }

  public static final String alphaRobotMAC = "00:80:2F:19:30:B7";
  public static final String betaRobotMAC = "00:80:2F:25:BC:FD";
}
