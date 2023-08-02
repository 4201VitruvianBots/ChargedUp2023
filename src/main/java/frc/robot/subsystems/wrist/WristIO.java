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
        public double percentOutput = 0;
        public double positionRadians = 0;
        public double angleDegrees = 0;

        public double outputVoltage = 0;
        public double outputCurrent = 0;
        
        public double simEncoderSign = 1;

        public double velocityDegreesPerSecond = 0;
    }

    /*Updates the loggable inputs */
    public default void updateInputs(WristIOInputs inputs) {}

    public default void setPercentOutput(double output){}

    public default void setSetpointPositionRadians (double desiredAngleRadians) {}

    public default void setSetpointangleDegrees(double angleDegrees) {}

    public default void setSetpointTrapezoidState(TrapezoidProfile.State state, double pos) {}

    public default void setPIDValues(double f, double p, double i, double d, double iZone) {}

    public default void simulationPeriodic() {}

    public default void setIValue(double value) {}

    public default void resetAngleDegrees(double angleDegrees) {}
} 
