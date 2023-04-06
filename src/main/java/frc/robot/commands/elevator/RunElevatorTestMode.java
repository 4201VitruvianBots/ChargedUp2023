// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.subsystems.Elevator;

public class RunElevatorTestMode extends CommandBase {
  private DoubleSubscriber kSetpointSub;
  private SimpleMotorFeedforward m_feedForward;
  private DoubleSubscriber kPSub;
  private DoubleSubscriber kISub;
  private DoubleSubscriber kDSub;
  private DoublePublisher kHeightPub;
  private DoubleSubscriber kMaxVelSub;
  private DoubleSubscriber kSSub;
  private DoubleSubscriber kMaxAccelSub;
  private DoubleSubscriber kVSub;
  private DoubleSubscriber kASub;
  private double testKP;
  private double testKI;
  private double testKA;
  private double testKV, kEncoderCountsPub, kDesiredHeightPub, kHeightInchesPub;
  private final Elevator m_elevator;
  private StringPublisher kDesiredStatePub, kClosedLoopModePub, currentCommandStatePub;
  private BooleanPublisher lowerLimitSwitchPub;
  private TrapezoidProfile.Constraints m_currentConstraints = ELEVATOR.m_slowConstraints;
  /** Creates a new RunElevatorTestMode. */
  public RunElevatorTestMode(Elevator elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable elevatorNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Elevator");

    // initialize Test Values
    kPSub = elevatorNtTab.getDoubleTopic("kP").subscribe(Constants.ELEVATOR.kP);
    kISub = elevatorNtTab.getDoubleTopic("kI").subscribe(Constants.ELEVATOR.kI);
    kDSub = elevatorNtTab.getDoubleTopic("kD").subscribe(Constants.ELEVATOR.kD);

    kMaxVelSub = elevatorNtTab.getDoubleTopic("Max Vel").subscribe(Constants.ELEVATOR.kMaxVel);
    kMaxAccelSub =
        elevatorNtTab.getDoubleTopic("Max Accel").subscribe(Constants.ELEVATOR.kMaxAccel);
    kSSub = elevatorNtTab.getDoubleTopic("kS").subscribe(Constants.ELEVATOR.kG);
    kVSub = elevatorNtTab.getDoubleTopic("kV").subscribe(Constants.ELEVATOR.kV);
    kASub = elevatorNtTab.getDoubleTopic("kA").subscribe(Constants.ELEVATOR.kA);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DriverStation.reportWarning("USING WRIST TEST MODE!", false);
    double maxVel = kMaxVelSub.get(0);
    double maxAccel = kMaxAccelSub.get(0);
    m_currentConstraints = new TrapezoidProfile.Constraints(maxVel, maxAccel);
    double kS = kSSub.get(Constants.ELEVATOR.kG);
    double kV = kVSub.get(Constants.ELEVATOR.kV);
    double kA = kASub.get(Constants.ELEVATOR.kA);
    double newTestKP = kPSub.get(0);

    if (testKP != newTestKP) {
      m_elevator.setTalonPIDvalues(0, 0, 0, 0, 0);
      testKP = newTestKP;
      m_feedForward = new SimpleMotorFeedforward(kS, kV, kA);
    }
    double newTestKI = kISub.get(0);
    if (testKP != newTestKP) {
      m_elevator.setTalonPIDvalues(0, 0, 0, 0, 0);
      testKI = newTestKI;
    }
    double newTestKV = kVSub.get(0);
    if (testKI != newTestKI) {
      m_elevator.setTalonPIDvalues(0, 0, 0, 0, 0);
      testKV = newTestKV;
    }
    double newTestKA = kASub.get(0);
    if (testKI != newTestKI) {
      m_elevator.setTalonPIDvalues(0, 0, 0, 0, 0);
      testKA = newTestKA;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
