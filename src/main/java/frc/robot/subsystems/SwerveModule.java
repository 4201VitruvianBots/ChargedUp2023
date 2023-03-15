// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SWERVEDRIVE.SWERVE_MODULE_POSITION;
import frc.robot.utils.CtreUtils;

public class SwerveModule extends SubsystemBase implements AutoCloseable {
  private final SWERVE_MODULE_POSITION m_modulePosition;
  private final int m_moduleNumber;
  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANCoder m_angleEncoder;
  private final double m_angleOffset;
  private double m_lastAngle;
  private Pose2d m_pose;
  private boolean m_initSuccess = false;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.SWERVEMODULE.ksDriveVoltSecondsPerMeter,
          Constants.SWERVEMODULE.kvDriveVoltSecondsSquaredPerMeter,
          Constants.SWERVEMODULE.kaDriveVoltSecondsSquaredPerMeter);

  private final FlywheelSim m_turnMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.1, 0.0001),
          Constants.SWERVEMODULE.kTurnGearbox,
          Constants.SWERVEMODULE.kTurningMotorGearRatio,
          VecBuilder.fill(0));

  private final FlywheelSim m_driveMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(1.5, 0.6),
          Constants.SWERVEMODULE.kDriveGearbox,
          Constants.SWERVEMODULE.kDriveMotorGearRatio);

  private double m_drivePercentOutput;
  private double m_turnPercentOutput;
  private double m_driveMotorSimDistance;
  private double m_turnMotorSimDistance;
  private final Timer m_simTimer = new Timer();
  private double m_lastSimTime = 0;
  private final double m_driveEncoderSimDirection;
  private final double m_turnEncoderSimDirection;

  // Logging setup

  private final DoubleLogEntry moduleTurnCurrentEntry;
  private final DoubleLogEntry moduleDriveCurrentEntry;

  private DoublePublisher moduleMotorHeadingPub, moduleEncoderHeadingPub;
  private BooleanPublisher moduleEncoderHealthPub;

  public SwerveModule(
      SWERVE_MODULE_POSITION modulePosition,
      TalonFX turnMotor,
      TalonFX driveMotor,
      CANCoder angleEncoder,
      double angleOffset) {
    m_modulePosition = modulePosition;
    m_moduleNumber = m_modulePosition.ordinal();
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_angleEncoder = angleEncoder;
    m_angleOffset = angleOffset;

    initModuleHeading();

    m_turnMotor.configFactoryDefault();
    m_turnMotor.configAllSettings(CtreUtils.generateTurnMotorConfig());
    m_turnMotor.setInverted(true);
    m_turnMotor.setSelectedSensorPosition(0);
    m_turnEncoderSimDirection = m_turnMotor.getInverted() ? -1.0 : 1.0;

    m_driveMotor.configFactoryDefault();
    m_driveMotor.configAllSettings(CtreUtils.generateDriveMotorConfig());
    m_driveMotor.setInverted(false);
    m_driveEncoderSimDirection = m_driveMotor.getInverted() ? -1.0 : 1.0;

    // m_angleEncoder.configMagnetOffset(m_angleOffset);
    m_lastAngle = getHeadingDegrees();

    initSmartDashboard();
    DataLog m_log = DataLogManager.getLog();
    moduleTurnCurrentEntry =
        new DoubleLogEntry(m_log, "/swerve/" + m_modulePosition.name() + "/turnCurrent");
    moduleDriveCurrentEntry =
        new DoubleLogEntry(m_log, "/swerve/" + m_modulePosition.name() + "/driveCurrent");

    if (RobotBase.isSimulation()) {
      m_simTimer.reset();
      m_simTimer.start();
    }
  }

  private void initModuleHeading() {
    if (RobotBase.isReal()) Timer.delay(0.2);
    m_angleEncoder.configFactoryDefault();
    m_angleEncoder.configAllSettings(CtreUtils.generateCanCoderConfig());
    resetAngleToAbsolute();

    // Check if the offset was applied properly. Delay to give it some time to set
    // TODO: This doesn't cover all edge cases
    if (RobotBase.isReal()) {
      Timer.delay(0.1);
      m_initSuccess =
          Math.abs(getHeadingDegrees() + m_angleOffset - m_angleEncoder.getAbsolutePosition())
              < 1.0;
    } else m_initSuccess = true;
  }

  public boolean getInitSuccess() {
    return m_initSuccess;
  }

  public SWERVE_MODULE_POSITION getModulePosition() {
    return m_modulePosition;
  }

  public void resetAngleToAbsolute() {
    double angle = m_angleEncoder.getAbsolutePosition() - m_angleOffset;
    m_turnMotor.setSelectedSensorPosition(
        angle / Constants.SWERVEMODULE.kTurningMotorDistancePerPulse);
  }

  public double getHeadingDegrees() {
    return m_turnMotor.getSelectedSensorPosition()
        * Constants.SWERVEMODULE.kTurningMotorDistancePerPulse;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getVelocityMetersPerSecond() {
    return m_driveMotor.getSelectedSensorVelocity()
        * Constants.SWERVEMODULE.kDriveMotorDistancePerPulse
        * 10;
  }

  public double getDriveMeters() {
    return m_driveMotor.getSelectedSensorPosition()
        * Constants.SWERVEMODULE.kDriveMotorDistancePerPulse;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput =
          desiredState.speedMetersPerSecond / Constants.SWERVEDRIVE.kMaxSpeedMetersPerSecond;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          desiredState.speedMetersPerSecond
              / (Constants.SWERVEMODULE.kDriveMotorDistancePerPulse * 10);
      m_driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (Constants.SWERVEDRIVE.kMaxSpeedMetersPerSecond * 0.01))
            ? m_lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents
    // Jittering.
    m_turnMotor.set(
        ControlMode.Position, angle / Constants.SWERVEMODULE.kTurningMotorDistancePerPulse);
    m_lastAngle = angle;

    m_drivePercentOutput = m_driveMotor.getMotorOutputPercent();
    m_turnPercentOutput = m_turnMotor.getMotorOutputPercent();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSecond(), getHeadingRotation2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public Pose2d getModulePose() {
    return m_pose;
  }

  public void setDriveNeutralMode(NeutralMode mode) {
    m_driveMotor.setNeutralMode(mode);
  }

  public void setTurnNeutralMode(NeutralMode mode) {
    m_turnMotor.setNeutralMode(mode);
  }

  private void initSmartDashboard() {
    var moduleTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Swerve");
    moduleEncoderHeadingPub =
        moduleTab.getDoubleTopic("Module (" + m_moduleNumber + ") Encoder Heading").publish();
    moduleTab
        .getDoubleTopic("Module (" + m_moduleNumber + ") Encoder Offset")
        .publish()
        .set(m_angleOffset);
    moduleEncoderHealthPub =
        moduleTab.getBooleanTopic("Module (" + m_moduleNumber + ") Encoder Health").publish();
    moduleMotorHeadingPub =
        moduleTab.getDoubleTopic("Module (" + m_moduleNumber + ") Motor Heading").publish();
  }

  private void updateSmartDashboard() {
    //    SmartDashboard.putNumber("Module " + m_moduleNumber + " error",
    // Math.abs(getHeadingDegrees() + m_angleOffset - m_angleEncoder.getAbsolutePosition()));

    moduleEncoderHeadingPub.set(m_angleEncoder.getAbsolutePosition());
    moduleMotorHeadingPub.set(getHeadingDegrees());
    moduleEncoderHealthPub.set(getInitSuccess());
  }

  public void updateLog() {
    // moduleTurnCurrentEntry.append(m_turnMotor.getMotorOutputVoltage());
    // moduleDriveCurrentEntry.append(m_driveMotor.getMotorOutputVoltage());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    updateLog();
  }

  @Override
  public void simulationPeriodic() {
    m_turnMotorSim.setInputVoltage(
        MathUtil.clamp(m_turnPercentOutput * RobotController.getBatteryVoltage(), -12, 12));
    m_driveMotorSim.setInputVoltage(
        MathUtil.clamp(m_drivePercentOutput * RobotController.getBatteryVoltage(), -12, 12));

    var currentTime = m_simTimer.get();
    m_turnMotorSim.update(currentTime - m_lastSimTime);
    m_driveMotorSim.update(currentTime - m_lastSimTime);
    m_lastSimTime = currentTime;

    Unmanaged.feedEnable(20);

    m_turnMotorSimDistance += m_turnMotorSim.getAngularVelocityRadPerSec() * 0.02;
    m_driveMotorSimDistance += m_driveMotorSim.getAngularVelocityRadPerSec() * 0.02;

    //    m_turnMotorSimDistance = Math.IEEEremainder(m_turnMotorSimDistance, 360);
    //    m_driveMotorSimDistance = Math.IEEEremainder(m_driveMotorSimDistance, 360);

    m_turnMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (m_turnEncoderSimDirection
                    * m_turnMotorSimDistance
                    / Constants.SWERVEMODULE.kTurningMotorDistancePerPulse));
    m_turnMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_turnEncoderSimDirection
                    * m_turnMotorSim.getAngularVelocityRadPerSec()
                    / (Constants.SWERVEMODULE.kTurningMotorDistancePerPulse * 10)));
    m_driveMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (m_driveEncoderSimDirection
                    * m_driveMotorSimDistance
                    / Constants.SWERVEMODULE.kDriveMotorDistancePerPulse));
    m_driveMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_driveEncoderSimDirection
                    * m_driveMotorSim.getAngularVelocityRadPerSec()
                    / (Constants.SWERVEMODULE.kDriveMotorDistancePerPulse * 10)));
  }

  @Override
  public void close() throws Exception {}
}
