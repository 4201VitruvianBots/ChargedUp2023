// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WRIST;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.wrist.Wrist;

public class RunWristTestMode extends CommandBase {
  private final Wrist m_wrist;
  private final StateHandler m_stateHandler;

  private final DoubleSubscriber kSetpointSub,
      kFSub,
      kPSub,
      kISub,
      kDSub,
      kIZoneSub,
      kGSub,
      kSSub,
      kVSub,
      kASub,
      kMaxVelSub,
      kMaxAccelSub;
  private double testKF,
      testKP,
      testKI,
      testKD,
      testKIZone,
      testKS,
      testKG,
      testKV,
      testKA,
      testMaxVel,
      testMaxAccel;

  /** Creates a new RunWristTestMode. */
  public RunWristTestMode(Wrist wrist, StateHandler stateHandler) {
    m_wrist = wrist;
    m_stateHandler = stateHandler;

    addRequirements(m_wrist);

    NetworkTable wristNtTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("WristControl");

    // initialize Test Values
    try {
      wristNtTab.getDoubleTopic("kSetpointDegrees").publish().set(0);

      wristNtTab.getDoubleTopic("kP").publish().set(WRIST.kP);
      wristNtTab.getDoubleTopic("kI").publish().set(WRIST.kI);
      wristNtTab.getDoubleTopic("kD").publish().set(WRIST.kD);
      wristNtTab.getDoubleTopic("kIZone").publish().set(0);

      wristNtTab
          .getDoubleTopic("Max Vel deg/s")
          .publish()
          .set(Units.radiansToDegrees(WRIST.kMaxVel));
      wristNtTab
          .getDoubleTopic("Max Accel deg/s^2")
          .publish()
          .set(Units.radiansToDegrees(WRIST.kMaxAccel));
      wristNtTab.getDoubleTopic("kS").publish().set(WRIST.FFkS);
      wristNtTab.getDoubleTopic("kG").publish().set(WRIST.kG);
      wristNtTab.getDoubleTopic("kV").publish().set(WRIST.FFkV);
      wristNtTab.getDoubleTopic("kA").publish().set(WRIST.kA);

    } catch (Exception m_ignored) {

    }

    kSetpointSub = wristNtTab.getDoubleTopic("kSetpointDegrees").subscribe(90);

    kFSub = wristNtTab.getDoubleTopic("kF").subscribe(0);
    kPSub = wristNtTab.getDoubleTopic("kP").subscribe(WRIST.kP);
    kISub = wristNtTab.getDoubleTopic("kI").subscribe(WRIST.kI);
    kDSub = wristNtTab.getDoubleTopic("kD").subscribe(WRIST.kD);
    kIZoneSub = wristNtTab.getDoubleTopic("kIZone").subscribe(0);

    kSSub = wristNtTab.getDoubleTopic("kS").subscribe(WRIST.FFkS);
    kGSub = wristNtTab.getDoubleTopic("kG").subscribe(WRIST.kG);
    kVSub = wristNtTab.getDoubleTopic("kV").subscribe(WRIST.FFkV);
    kASub = wristNtTab.getDoubleTopic("kA").subscribe(WRIST.kA);

    kMaxVelSub = wristNtTab.getDoubleTopic("Max Vel deg/s").subscribe(WRIST.kMaxVel);
    kMaxAccelSub = wristNtTab.getDoubleTopic("Max Accel deg/s^2").subscribe(WRIST.kMaxAccel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Disable the state handler
    m_stateHandler.disable();

    m_wrist.setUserSetpoint(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriverStation.reportWarning("USING WRIST TEST MODE!", false);
    double newSetpoint = Units.degreesToRadians(kSetpointSub.get(90));

    double newKF = kFSub.get(0);
    double newKP = kPSub.get(WRIST.kP);
    double newKI = kISub.get(WRIST.kI);
    double newKD = kDSub.get(WRIST.kD);
    double newIZone = kIZoneSub.get(0);

    double newKS = kSSub.get(WRIST.FFkS);
    double newKG = kGSub.get(WRIST.kG);
    double newKV = kVSub.get(WRIST.FFkV);
    double newKA = kASub.get(WRIST.kA);

    double newMaxVel = Units.degreesToRadians(kMaxVelSub.get(WRIST.kMaxVel));
    double newMaxAccel = Units.degreesToRadians(kMaxAccelSub.get(WRIST.kMaxAccel));

    if (testKF != newKF
        || (testKP != newKP || testKI != newKI || testKD != newKD || newIZone != newIZone)) {
      m_wrist.setPIDValues(newKF, newKP, newKI, newKD, newIZone);
      testKF = newKF;
      testKP = newKP;
      testKI = newKI;
      testKD = newKD;
      testKIZone = newIZone;
    }
    if (testKS != newKS || testKG != newKG || testKV != newKV || testKA != newKA) {
      m_wrist.setArmMotorFeedForward(newKS, newKG, newKV, newKA);
      testKS = newKS;
      testKG = newKG;
      testKV = newKV;
      testKA = newKA;
    }

    if (testMaxVel != newMaxVel || testMaxAccel != newMaxAccel) {
      m_wrist.setTrapezoidalConstraints(newMaxVel, newMaxAccel);
      testMaxVel = newMaxVel;
      testMaxAccel = newMaxAccel;
    }

    m_wrist.setSetpointPositionRadians(newSetpoint);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setUserSetpoint(false);

    // Re-enable the state handler
    m_stateHandler.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
