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
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.Elevator;

public class RunElevatorTestMode extends CommandBase {
  private final Elevator m_elevator;

  private DoubleSubscriber kSetpointSub,
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
  public RunElevatorTestMode(Elevator elevator) {
    m_elevator = elevator;

    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable elevatorNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ElevatorControls");

    // initialize Test Values
    kSetpointSub = elevatorNtTab.getDoubleTopic("kSetpointInches").subscribe(0);

    kFSub = elevatorNtTab.getDoubleTopic("kF").subscribe(0);
    kPSub = elevatorNtTab.getDoubleTopic("kP").subscribe(ELEVATOR.kP);
    kISub = elevatorNtTab.getDoubleTopic("kI").subscribe(ELEVATOR.kI);
    kDSub = elevatorNtTab.getDoubleTopic("kD").subscribe(ELEVATOR.kD);
    kIZoneSub = elevatorNtTab.getDoubleTopic("kIZone").subscribe(0);

    kGSub = elevatorNtTab.getDoubleTopic("kG").subscribe(ELEVATOR.kG);
    kVSub = elevatorNtTab.getDoubleTopic("kV").subscribe(ELEVATOR.kV);
    kASub = elevatorNtTab.getDoubleTopic("kA").subscribe(ELEVATOR.kA);

    kMaxVelSub = elevatorNtTab.getDoubleTopic("Max Vel").subscribe(ELEVATOR.kMaxVel);
    kMaxAccelSub =
        elevatorNtTab.getDoubleTopic("Max Accel").subscribe(Constants.ELEVATOR.kMaxAccel);

    m_elevator.setUserSetpoint(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriverStation.reportWarning("USING WRIST TEST MODE!", false);
    double newSetpoint = Units.inchesToMeters(kSetpointSub.get(0));

    double newKF = kFSub.get(0);
    double newKP = kPSub.get(ELEVATOR.kP);
    double newKI = kISub.get(ELEVATOR.kI);
    double newKD = kDSub.get(ELEVATOR.kD);
    double newIZone = kIZoneSub.get(0);

    double newKG = kGSub.get(Constants.ELEVATOR.kG);
    double newKV = kVSub.get(Constants.ELEVATOR.kV);
    double newKA = kASub.get(Constants.ELEVATOR.kA);

    double newMaxVel = kMaxVelSub.get(ELEVATOR.kMaxVel);
    double newMaxAccel = kMaxAccelSub.get(ELEVATOR.kMaxAccel);

    if (testKF != newKF
        || (testKP != newKP || testKI != newKI || testKD != newKD || newIZone != newIZone)) {
      m_elevator.setTalonPIDvalues(newKF, newKP, newKI, newKD, newIZone);
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
      testMaxVel = newKG;
      testMaxAccel = newKV;
    }

    m_elevator.setDesiredPositionMeters(newSetpoint);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setUserSetpoint(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
