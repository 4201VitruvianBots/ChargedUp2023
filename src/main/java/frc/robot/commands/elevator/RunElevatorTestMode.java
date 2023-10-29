// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.elevator.Elevator;

public class RunElevatorTestMode extends CommandBase {
  private final Elevator m_elevator;
  private final StateHandler m_stateHandler;

  private final DoubleSubscriber kSetpointSub,
      kFSub,
      kPSub,
      kISub,
      kDSub,
      kIZoneSub,
      kGSub,
      kVSub,
      kASub,
      kMaxVelSub,
      kMaxAccelSub;
  private double testKF,
      testKP,
      testKI,
      testKD,
      testKIZone,
      testKG,
      testKV,
      testKA,
      testMaxVel,
      testMaxAccel;

  /** Creates a new RunElevatorTestMode. */
  public RunElevatorTestMode(Elevator elevator, StateHandler stateHandler) {
    m_elevator = elevator;
    m_stateHandler = stateHandler;

    addRequirements(m_elevator);

    NetworkTable elevatorNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ElevatorControls");

    // initialize Test Values
    try {
      elevatorNtTab.getDoubleTopic("kSetpointInches").publish().set(0);

      elevatorNtTab.getDoubleTopic("kP").publish().set(ELEVATOR.kP);
      elevatorNtTab.getDoubleTopic("kI").publish().set(ELEVATOR.kI);
      elevatorNtTab.getDoubleTopic("kD").publish().set(ELEVATOR.kD);
      elevatorNtTab.getDoubleTopic("kIZone").publish().set(0);

      elevatorNtTab
          .getDoubleTopic("Max Vel in/s")
          .publish()
          .set(Units.metersToInches(ELEVATOR.kMaxVel));
      elevatorNtTab
          .getDoubleTopic("Max Accel in/s^2")
          .publish()
          .set(Units.metersToInches(ELEVATOR.kMaxAccel));
      elevatorNtTab.getDoubleTopic("kG").publish().set(ELEVATOR.kG);
      elevatorNtTab.getDoubleTopic("kV").publish().set(ELEVATOR.kV);
      elevatorNtTab.getDoubleTopic("kA").publish().set(ELEVATOR.kA);
    } catch (Exception m_ignored) {

    }

    kSetpointSub = elevatorNtTab.getDoubleTopic("kSetpointInches").subscribe(0);

    kFSub = elevatorNtTab.getDoubleTopic("kF").subscribe(0);
    kPSub = elevatorNtTab.getDoubleTopic("kP").subscribe(ELEVATOR.kP);
    kISub = elevatorNtTab.getDoubleTopic("kI").subscribe(ELEVATOR.kI);
    kDSub = elevatorNtTab.getDoubleTopic("kD").subscribe(ELEVATOR.kD);
    kIZoneSub = elevatorNtTab.getDoubleTopic("kIZone").subscribe(0);

    kGSub = elevatorNtTab.getDoubleTopic("kG").subscribe(ELEVATOR.kG);
    kVSub = elevatorNtTab.getDoubleTopic("kV").subscribe(ELEVATOR.kV);
    kASub = elevatorNtTab.getDoubleTopic("kA").subscribe(ELEVATOR.kA);

    kMaxVelSub = elevatorNtTab.getDoubleTopic("Max Vel in/s").subscribe(ELEVATOR.kMaxVel);
    kMaxAccelSub = elevatorNtTab.getDoubleTopic("Max Accel in/s^2").subscribe(ELEVATOR.kMaxAccel);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Disable the state handler
    m_stateHandler.disable();

    m_elevator.setUserSetpoint(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriverStation.reportWarning("USING ELEVATOR TEST MODE!", false);
    double newSetpoint = Units.inchesToMeters(kSetpointSub.get(0));

    double newKF = kFSub.get(0);
    double newKP = kPSub.get(ELEVATOR.kP);
    double newKI = kISub.get(ELEVATOR.kI);
    double newKD = kDSub.get(ELEVATOR.kD);
    double newIZone = kIZoneSub.get(0);

    double newKG = kGSub.get(ELEVATOR.kG);
    double newKV = kVSub.get(ELEVATOR.kV);
    double newKA = kASub.get(ELEVATOR.kA);

    double newMaxVel = Units.inchesToMeters(kMaxVelSub.get(ELEVATOR.kMaxVel));
    double newMaxAccel = Units.inchesToMeters(kMaxAccelSub.get(ELEVATOR.kMaxAccel));

    if (testKF != newKF
        || testKP != newKP
        || testKI != newKI
        || testKD != newKD
        || newIZone != newIZone) {
      m_elevator.setPIDvalues(newKF, newKP, newKI, newKD, newIZone);
      testKF = newKF;
      testKP = newKP;
      testKI = newKI;
      testKD = newKD;
      testKIZone = newIZone;
    }

    if (testKG != newKG || testKV != newKV || testKA != newKA) {
      m_elevator.setSimpleMotorFeedForward(newKG, newKV, newKA);
      testKG = newKG;
      testKV = newKV;
      testKA = newKA;
    }

    if (testMaxVel != newMaxVel || testMaxAccel != newMaxAccel) {
      m_elevator.setTrapezoidalConstraints(newMaxVel, newMaxAccel);
      testMaxVel = newMaxVel;
      testMaxAccel = newMaxAccel;
    }

    m_elevator.setDesiredPositionMeters(newSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setUserSetpoint(false);

    // Re-enable the state handler
    m_stateHandler.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
