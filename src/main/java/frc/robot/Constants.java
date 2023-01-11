// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static class USB {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;
  }

  public final class Pnuematics {

  }

  public final class CAN {

  }

  public final class Elevator {
    public static final int elevatorMotorLeft = 21;
    public static final int elevatorMotorRight = 22;
  }

  public final class Claw {

  }

  public final class LED {

  }

  public final class Vision {

  }

  public final class DriveConstants{
    public final class AutoConstants{}

    public final class ModuleConstants{}
  }


}
