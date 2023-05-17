// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double percentOutput = 0;
        public double velocityMetersPerSec = 0;
        public double heightEncoderCounts = 0;
        public double heightMeters = 0;

        public double outputVoltage = 0;
        public double outputCurrent = 0;

        public int simEncoderSign = 1;
        public NeutralMode neutralMode = NeutralMode.Brake;

    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    // Setting the raw output of the motors
    public default void setPercentOutput(double output) {
    }

    // Sets the calculated trapezoid state of the motors
    public default void setSetpointTrapezoidState(TrapezoidProfile.State state) {
    }
}
