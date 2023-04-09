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
import frc.robot.subsystems.Wrist;

public class RunWristTestMode extends CommandBase {
  private final Wrist m_wrist;

  private DoubleSubscriber kSetpointSub,
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
  public RunWristTestMode(Wrist wrist) {
    m_wrist = wrist;

    addRequirements(m_wrist);

    NetworkTable wristNtTab =
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("WristControl");

    // initialize Test Values
    kSetpointSub = wristNtTab.getDoubleTopic("kSetpointDegrees").subscribe(0);

    kFSub = wristNtTab.getDoubleTopic("kF").subscribe(0);
    kPSub = wristNtTab.getDoubleTopic("kP").subscribe(WRIST.kP);
    kISub = wristNtTab.getDoubleTopic("kI").subscribe(WRIST.kI);
    kDSub = wristNtTab.getDoubleTopic("kD").subscribe(WRIST.kD);
    kIZoneSub = wristNtTab.getDoubleTopic("kIZone").subscribe(0);

    kSSub = wristNtTab.getDoubleTopic("kS").subscribe(WRIST.FFkS);
    kGSub = wristNtTab.getDoubleTopic("kG").subscribe(WRIST.kG);
    kVSub = wristNtTab.getDoubleTopic("kV").subscribe(WRIST.FFkV);
    kASub = wristNtTab.getDoubleTopic("kA").subscribe(WRIST.kA);

    kMaxVelSub = wristNtTab.getDoubleTopic("Max Vel").subscribe(WRIST.kMaxSlowVel);
    kMaxAccelSub = wristNtTab.getDoubleTopic("Max Accel").subscribe(WRIST.kMaxSlowAccel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setUserSetpoint(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriverStation.reportWarning("USING WRIST TEST MODE!", false);
    double newSetpoint = Units.degreesToRadians(kSetpointSub.get(0));

    double newKF = kFSub.get(0);
    double newKP = kPSub.get(WRIST.kP);
    double newKI = kISub.get(WRIST.kI);
    double newKD = kDSub.get(WRIST.kD);
    double newIZone = kIZoneSub.get(0);

    double newKS = kSSub.get(WRIST.FFkS);
    double newKG = kGSub.get(WRIST.kG);
    double newKV = kVSub.get(WRIST.FFkV);
    double newKA = kASub.get(WRIST.kA);

    double newMaxVel = kMaxVelSub.get(WRIST.kMaxSlowVel);
    double newMaxAccel = kMaxAccelSub.get(WRIST.kMaxSlowAccel);

    if (testKF != newKF
        || (testKP != newKP || testKI != newKI || testKD != newKD || newIZone != newIZone)) {
      m_wrist.setTalonPIDvalues(newKF, newKP, newKI, newKD, newIZone);
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
      testMaxVel = newKG;
      testMaxAccel = newKV;
    }

    m_wrist.setSetpointPositionRadians(newSetpoint);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.setUserSetpoint(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
