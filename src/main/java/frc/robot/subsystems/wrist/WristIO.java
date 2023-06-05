// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double wristPercentOutput = 0;
        public double positionRadians = 0;

        public double wristOutputVoltage = 0;
        public double wristOutputCurrent = 0;
        
        public double simEncoderSign = 1;


    }

    /*Updates the loggable inputs */
    public default void updateInputs(WristIOInputs inputs) {}

    public default void setPercentOutput(double output){}

    public default void setSetpointTrapezoidState(TrapezoidProfile.State state) {}

    public default void setPIDValues(double f, double p, double i, double d, double iZone) {}
} 
