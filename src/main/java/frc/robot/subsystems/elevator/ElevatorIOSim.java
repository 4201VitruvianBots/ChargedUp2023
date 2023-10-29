// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.StateHandler.m_elevatorRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.ELEVATOR.THRESHOLD;
import frc.robot.subsystems.StateHandler;

/** Add your docs here. */
public class ElevatorIOSim extends ElevatorIOReal {

  private static int m_simEncoderSign = 1;

  // Mechanism2d visualization setup
  private MechanismLigament2d m_elevatorLigament2d;

  // Simulation setup
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          ELEVATOR.gearbox,
          ELEVATOR.gearRatio,
          ELEVATOR.massKg,
          ELEVATOR.drumRadiusMeters,
          THRESHOLD.ABSOLUTE_MIN.get(),
          THRESHOLD.ABSOLUTE_MAX.get(),
          true);

  public ElevatorIOSim() {
    super();

    m_simEncoderSign = elevatorMotors[0].getInverted() ? -1 : 1;

    try {
      m_elevatorLigament2d =
          m_elevatorRoot2d.append(
              new MechanismLigament2d(
                  "Elevator", 0 + ELEVATOR.carriageDistance, ELEVATOR.mech2dAngleDegrees));
      m_elevatorLigament2d.setColor(new Color8Bit(180, 0, 0)); // Red
    } catch (Exception m_ignored) {

    }
  }

  // Returns the ligament of the elevator to update in StateHandler
  public MechanismLigament2d getLigament() {
    return m_elevatorLigament2d;
  }

  public void setLigament(MechanismLigament2d ligament) {
    m_elevatorLigament2d = ligament;
  }

  @Override
  public void simulationPeriodic() {
    elevatorSim.setInput(MathUtil.clamp(elevatorMotors[0].getMotorOutputVoltage(), -12, 12));

    double dt = StateHandler.getSimDt();
    elevatorSim.update(dt);

    // Internally sets the position of the motors in encoder counts based on our current height in
    // meters
    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (m_simEncoderSign
                    * elevatorSim.getPositionMeters()
                    / ELEVATOR.encoderCountsToMeters));

    // Internally sets the velocity of the motors in encoder counts per 100 ms based on our velocity
    // in meters per second (1000 ms)
    elevatorMotors[0]
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_simEncoderSign
                    * elevatorSim.getVelocityMetersPerSecond()
                    / ELEVATOR.encoderCountsToMeters
                    * 10));

    // Sets the simulated voltage of the roboRio based on our current draw from the elevator
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    elevatorMotors[0].getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
    elevatorMotors[1].getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());

    if (m_elevatorLigament2d != null)
      m_elevatorLigament2d.setLength(elevatorSim.getPositionMeters());
  }

  @Override
  // Safely closes the subsystem
  public void close() throws Exception {
    //    lowerLimitSwitch.close();
    if (m_elevatorLigament2d != null) m_elevatorLigament2d.close();
  }
}
