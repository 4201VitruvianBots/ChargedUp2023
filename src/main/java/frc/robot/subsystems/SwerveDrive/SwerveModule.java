// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
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
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.SWERVE_MODULE_POSITION;
import frc.robot.subsystems.StateHandler;
import frc.robot.Constants.SWERVE_MODULE;
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

  private final boolean m_limitCanUtil = STATE_HANDLER.limitCanUtilization;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SWERVE_MODULE.ksDriveVoltSecondsPerMeter,
          SWERVE_MODULE.kvDriveVoltSecondsSquaredPerMeter,
          SWERVE_MODULE.kaDriveVoltSecondsSquaredPerMeter);

  private final FlywheelSim m_turnMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.25, 0.000001),
          SWERVE_MODULE.kTurnGearbox,
          SWERVE_MODULE.kTurningMotorGearRatio,
          VecBuilder.fill(0));

  private final FlywheelSim m_driveMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.8, 0.6),
          SWERVE_MODULE.kDriveGearbox,
          SWERVE_MODULE.kDriveMotorGearRatio);

  private double m_driveMotorSimDistance;
  private double m_turnMotorSimDistance;

  private final int m_driveEncoderSimSign;
  private final int m_turnEncoderSimSign;

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
    m_turnEncoderSimSign = m_turnMotor.getInverted() ? -1 : 1;

    m_driveMotor.configFactoryDefault();
    m_driveMotor.configAllSettings(CtreUtils.generateDriveMotorConfig());
    m_driveMotor.setInverted(false);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turnMotor.setNeutralMode(NeutralMode.Brake);

    m_driveEncoderSimSign = m_driveMotor.getInverted() ? -1 : 1;

    // m_angleEncoder.configMagnetOffset(m_angleOffset);
    m_lastAngle = getHeadingDegrees();

    initSmartDashboard();
    DataLog m_log = DataLogManager.getLog();
    moduleTurnCurrentEntry =
        new DoubleLogEntry(m_log, "/swerve/" + m_modulePosition.name() + "/turnCurrent");
    moduleDriveCurrentEntry =
        new DoubleLogEntry(m_log, "/swerve/" + m_modulePosition.name() + "/driveCurrent");

    // To distinguish modules in CommandScheduler
    setName("SwerveModule_" + m_modulePosition.ordinal());
  }

  private void initModuleHeading() {
    if (RobotBase.isReal()) Timer.delay(0.2);
    m_angleEncoder.configFactoryDefault();
    m_angleEncoder.configAllSettings(CtreUtils.generateCanCoderConfig());
    m_angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    m_angleEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
    resetAngleToAbsolute();

    // Check if the offset was applied properly. Delay to give it some time to set
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
    resetAngle(0);
  }

  public void resetAngle(double angle) {
    double newAngle = m_angleEncoder.getAbsolutePosition() - m_angleOffset + angle;
    m_turnMotor.setSelectedSensorPosition(newAngle / SWERVE_MODULE.kTurningMotorDistancePerPulse);
  }

  public double getHeadingDegrees() {
    return m_turnMotor.getSelectedSensorPosition() * SWERVE_MODULE.kTurningMotorDistancePerPulse;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getVelocityMetersPerSecond() {
    return m_driveMotor.getSelectedSensorVelocity()
        * SWERVE_MODULE.kDriveMotorDistancePerPulse
        * 10;
  }

  public double getDriveMeters() {
    return m_driveMotor.getSelectedSensorPosition() * SWERVE_MODULE.kDriveMotorDistancePerPulse;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput =
          desiredState.speedMetersPerSecond / SWERVE_DRIVE.kMaxSpeedMetersPerSecond;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity =
          desiredState.speedMetersPerSecond / (SWERVE_MODULE.kDriveMotorDistancePerPulse * 10);
      m_driveMotor.set(
          ControlMode.Velocity,
          velocity,
          DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.01))
            ? m_lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents
    // Jittering.
    m_turnMotor.set(ControlMode.Position, angle / SWERVE_MODULE.kTurningMotorDistancePerPulse);
    m_lastAngle = angle;
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
    moduleMotorHeadingPub.set(getHeadingDegrees());
    moduleEncoderHealthPub.set(getInitSuccess());
    if (!m_limitCanUtil) {
      moduleEncoderHeadingPub.set(m_angleEncoder.getAbsolutePosition());
    }
  }

  public void updateLog() {
    moduleTurnCurrentEntry.append(m_turnMotor.getMotorOutputVoltage());
    moduleDriveCurrentEntry.append(m_driveMotor.getMotorOutputVoltage());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    //    updateLog();
  }

  @Override
  public void simulationPeriodic() {
    m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_turnMotor.getMotorOutputVoltage(), -12, 12));
    m_driveMotorSim.setInputVoltage(MathUtil.clamp(m_driveMotor.getMotorOutputVoltage(), -12, 12));

    double dt = StateHandler.getSimDt();
    m_turnMotorSim.update(dt);
    m_driveMotorSim.update(dt);

    m_turnMotorSimDistance += m_turnMotorSim.getAngularVelocityRadPerSec() * dt;
    m_driveMotorSimDistance += m_driveMotorSim.getAngularVelocityRadPerSec() * dt;

    Unmanaged.feedEnable(20);

    m_turnMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (m_turnEncoderSimSign
                    * m_turnMotorSimDistance
                    / SWERVE_MODULE.kTurningMotorDistancePerPulse));
    m_turnMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_turnEncoderSimSign
                    * m_turnMotorSim.getAngularVelocityRadPerSec()
                    / (SWERVE_MODULE.kTurningMotorDistancePerPulse * 10)));
    m_driveMotor
        .getSimCollection()
        .setIntegratedSensorRawPosition(
            (int)
                (m_driveEncoderSimSign
                    * m_driveMotorSimDistance
                    / SWERVE_MODULE.kDriveMotorDistancePerPulse));
    m_driveMotor
        .getSimCollection()
        .setIntegratedSensorVelocity(
            (int)
                (m_driveEncoderSimSign
                    * m_driveMotorSim.getAngularVelocityRadPerSec()
                    / (SWERVE_MODULE.kDriveMotorDistancePerPulse * 10)));

    m_turnMotor.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
    m_driveMotor.getSimCollection().setBusVoltage(RobotController.getBatteryVoltage());
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
