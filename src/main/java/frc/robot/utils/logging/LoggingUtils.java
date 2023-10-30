// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.logging;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A utility class that provides functions to automatically publish values to AdvantageKit. */
public class LoggingUtils {
    /**
    * Pushes a number value to SmartDashboard and AdvantageKit simultaneously.
    * Removes whitespace from key to be compatible with AdvantageKit naming standards.
    *
    * @param key the key to be assigned to
    * @param value the value that will be assigned
    */
    public static void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
        Logger.getInstance().recordOutput("LoggingUtils/SmartDashboard/" + key.replaceAll("\\s+",""), value);
    }

    /**
    * Pushes a boolean value to SmartDashboard and AdvantageKit simultaneously.
    * Removes whitespace from key to be compatible with AdvantageKit naming standards.
    *
    * @param key the key to be assigned to
    * @param value the value that will be assigned
    */
    public static void putBoolean(String key, Boolean value) {
        SmartDashboard.putBoolean(key, value);
        Logger.getInstance().recordOutput("LoggingUtils/SmartDashboard/" + key.replaceAll("\\s+",""), value);
    }

    /**
    * Pushes a string to SmartDashboard and AdvantageKit simultaneously.
    * Removes whitespace from key to be compatible with AdvantageKit naming standards.
    *
    * @param key the key to be assigned to
    * @param value the value that will be assigned
    */
    public static void putString(String key, String value) {
        SmartDashboard.putString(key, value);
        Logger.getInstance().recordOutput("LoggingUtils/SmartDashboard/" + key.replaceAll("\\s+",""), value);
    }
}
