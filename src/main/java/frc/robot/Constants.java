// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

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

  public static final class Vision {
    public enum CAMERA_TYPE {
      OAK,
      LIMELIGHT,
      PHOTONVISION
    }

    public enum CAMERA_POSITION {
      INTAKE,
      OUTTAKE,
      FORWARD_LOCALIZER,
      REAR_LOCALIZER
    }

    public enum SERVER_IPS {
      INTAKE("10.42.1.10"),
      FORWARD_LOCALIZER("10.42.1.11"),
      REAR_LOCALIZER("10.42.1.12");

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

  public enum SwerveDriveModulePosition {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }

  // Switches between constants class depending on the MAC address of the roboRIO we're running on
  
  public static final String alphaRobotMAC = "00:00:00:00:00:01";
  public static final String betaRobotMAC = "00:00:00:00:00:02";

  public static ConstantsAlpha constants;
  
  public Constants() {

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    String mac = inst.getTable("RIO-Info").getEntry("MAC").getString("N/A");
    if (mac == alphaRobotMAC) {
      final ConstantsAlpha constants = new ConstantsAlpha();
    } 
    else if (mac == betaRobotMAC) {
      final ConstantsAlpha constants = new ConstantsBeta();
    }
    else {
      final ConstantsAlpha constants = new ConstantsAlpha();
    }

  }
}
