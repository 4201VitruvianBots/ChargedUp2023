// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.SWERVE_MODULE_POSITION;
import frc.robot.utils.ModuleMap;
import java.util.HashMap;
import java.util.Map;

public class SwerveDrive extends SubsystemBase implements AutoCloseable {

  private final HashMap<SWERVE_MODULE_POSITION, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
              SWERVE_MODULE_POSITION.FRONT_LEFT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.FRONT_LEFT,
                      new TalonFX(CAN.frontLeftTurnMotor),
                      new TalonFX(CAN.frontLeftDriveMotor),
                      new CANCoder(CAN.frontLeftCanCoder),
                      SWERVE_DRIVE.frontLeftCANCoderOffset),
              SWERVE_MODULE_POSITION.FRONT_RIGHT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.FRONT_RIGHT,
                      new TalonFX(CAN.frontRightTurnMotor),
                      new TalonFX(CAN.frontRightDriveMotor),
                      new CANCoder(CAN.frontRightCanCoder),
                      SWERVE_DRIVE.frontRightCANCoderOffset),
              SWERVE_MODULE_POSITION.BACK_LEFT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.BACK_LEFT,
                      new TalonFX(CAN.backLeftTurnMotor),
                      new TalonFX(CAN.backLeftDriveMotor),
                      new CANCoder(CAN.backLeftCanCoder),
                      SWERVE_DRIVE.backLeftCANCoderOffset),
              SWERVE_MODULE_POSITION.BACK_RIGHT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.BACK_RIGHT,
                      new TalonFX(CAN.backRightTurnMotor),
                      new TalonFX(CAN.backRightDriveMotor),
                      new CANCoder(CAN.backRightCanCoder),
                      SWERVE_DRIVE.backRightCANCoderOffset)));

  private final Pigeon2 m_pigeon = new Pigeon2(CAN.pigeon, "rio");
  private double m_rollOffset;

  private final boolean m_limitCanUtil = STATE_HANDLER.limitCanUtilization;

  private final SwerveDrivePoseEstimator m_odometry;
  private boolean m_simOverride = false; // DO NOT MAKE FINAL. WILL BREAK UNIT TESTS
  private final Timer m_simTimer = new Timer();
  private double m_lastSimTime = 0;
  private double m_simYaw;
  private double m_simRoll;
  private DoublePublisher pitchPub, rollPub, yawPub, odometryXPub, odometryYPub, odometryYawPub;

  private boolean useHeadingTarget = false;
  private double m_desiredHeadingRadians;

  private final PIDController m_xController =
      new PIDController(SWERVE_DRIVE.kP_X, SWERVE_DRIVE.kI_X, SWERVE_DRIVE.kD_X);
  private final PIDController m_yController =
      new PIDController(SWERVE_DRIVE.kP_Y, SWERVE_DRIVE.kI_Y, SWERVE_DRIVE.kD_Y);
  private final PIDController m_turnController =
      new PIDController(SWERVE_DRIVE.kP_Theta, SWERVE_DRIVE.kI_Theta, SWERVE_DRIVE.kD_Theta);

  private double m_rotationOutput;

  ChassisSpeeds chassisSpeeds;
  private double m_maxVelocity = SWERVE_DRIVE.kMaxSpeedMetersPerSecond;

  public SwerveDrive() {
    m_pigeon.configFactoryDefault();
    m_pigeon.setYaw(0);
    m_odometry =
        new SwerveDrivePoseEstimator(
            SWERVE_DRIVE.kSwerveKinematics,
            getHeadingRotation2d(),
            getSwerveDriveModulePositionsArray(),
            new Pose2d());

    if (RobotBase.isReal()) {
      Timer.delay(1);
      resetModulesToAbsolute();
    } else {
      m_simTimer.reset();
      m_simTimer.start();
    }
    initSmartDashboard();
  }

  private void resetModulesToAbsolute() {
    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.resetAngleToAbsolute();
  }

  public void drive(
      double throttle,
      double strafe,
      double rotation,
      boolean isFieldRelative,
      boolean isOpenLoop) {
    throttle *= m_maxVelocity;
    strafe *= m_maxVelocity;
    rotation *= SWERVE_DRIVE.kMaxRotationRadiansPerSecond;

    /** Setting field vs Robot Relative */
    if (useHeadingTarget) {
      rotation = m_rotationOutput;
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getHeadingRotation2d());
    } else {
      chassisSpeeds =
          isFieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  throttle, strafe, rotation, getHeadingRotation2d())
              : new ChassisSpeeds(throttle, strafe, rotation);
    }

    Map<SWERVE_MODULE_POSITION, SwerveModuleState> moduleStates =
        ModuleMap.of(SWERVE_DRIVE.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), m_maxVelocity);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
  }

  /*
   * Set robot heading to a clear target
   */

  public void setRobotHeadingRadians(double radians) {
    m_desiredHeadingRadians = MathUtil.inputModulus(radians, -Math.PI, Math.PI);
  }

  public void calculateRotationSpeed() {
    if (Math.abs(getHeadingRotation2d().getRadians() - m_desiredHeadingRadians)
        > Units.degreesToRadians(1))
      m_rotationOutput =
          m_turnController.calculate(getHeadingRotation2d().getRadians(), m_desiredHeadingRadians);
    else m_rotationOutput = 0;
  }

  /*
   * ability to let head of swerve drive face the target
   */
  public void enableHeadingTarget(boolean enable) {
    useHeadingTarget = enable;
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocity);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
    var states = SWERVE_DRIVE.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    setSwerveModuleStates(states, false);
  }

  public void setOdometry(Pose2d pose) {
    m_pigeon.setYaw(pose.getRotation().getDegrees());
    m_odometry.resetPosition(getHeadingRotation2d(), getSwerveDriveModulePositionsArray(), pose);
  }

  public void setRollOffset() {
    m_rollOffset = -m_pigeon.getRoll(); // -2.63
  }

  public double getRollOffsetDegrees() {
    return m_rollOffset;
  }

  public double getPitchDegrees() {
    return m_pigeon.getPitch();
  }

  public double getRollDegrees() {
    if (m_simOverride) return m_simRoll;
    else return m_pigeon.getRoll();
  }

  public double getHeadingDegrees() {
    return m_pigeon.getYaw();
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getEstimatedPosition();
  }

  public SwerveModule getSwerveModule(SWERVE_MODULE_POSITION modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public Map<SWERVE_MODULE_POSITION, SwerveModuleState> getModuleStates() {
    Map<SWERVE_MODULE_POSITION, SwerveModuleState> map = new HashMap<>();
    for (SWERVE_MODULE_POSITION i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getState());
    return map;
  }

  public Map<SWERVE_MODULE_POSITION, SwerveModulePosition> getModulePositions() {
    Map<SWERVE_MODULE_POSITION, SwerveModulePosition> map = new HashMap<>();
    for (SWERVE_MODULE_POSITION i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getPosition());
    return map;
  }

  public SwerveModulePosition[] getSwerveDriveModulePositionsArray() {
    return ModuleMap.orderedValues(getModulePositions(), new SwerveModulePosition[0]);
  }

  public Map<SWERVE_MODULE_POSITION, Pose2d> getModulePoses() {
    Map<SWERVE_MODULE_POSITION, Pose2d> map = new HashMap<>();
    for (SWERVE_MODULE_POSITION i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getModulePose());
    return map;
  }

  public boolean getModuleInitStatus() {
    for (SWERVE_MODULE_POSITION i : m_swerveModules.keySet()) {
      if (!m_swerveModules.get(i).getInitSuccess()) {
        return false;
      }
    }
    return true;
  }

  public PIDController getXPidController() {
    return m_xController;
  }

  public PIDController getYPidController() {
    return m_yController;
  }

  public PIDController getThetaPidController() {
    return m_turnController;
  }

  public void setNeutralMode(NeutralMode mode) {
    for (SwerveModule module : m_swerveModules.values()) {
      //      module.setDriveNeutralMode(mode);
      module.setTurnNeutralMode(mode);
    }
  }

  public void setMaxVelocity(double mps) {
    m_maxVelocity = mps;
  }

  public SwerveDrivePoseEstimator getOdometry() {
    return m_odometry;
  }

  public void resetGyro() {
    m_pigeon.setYaw(0);
    m_pigeon.setAccumZAngle(0);
  }

  public void updateOdometry() {
    m_odometry.update(getHeadingRotation2d(), getSwerveDriveModulePositionsArray());

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
      Transform2d moduleTransform =
          new Transform2d(
              SWERVE_DRIVE.kModuleTranslations.get(module.getModulePosition()),
              module.getHeadingRotation2d());
      module.setModulePose(getPoseMeters().transformBy(moduleTransform));
    }
  }

  private void initSmartDashboard() {
    SmartDashboard.putData(this);

    var swerveTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Swerve");
    pitchPub = swerveTab.getDoubleTopic("Pitch").publish();
    rollPub = swerveTab.getDoubleTopic("Roll").publish();
    yawPub = swerveTab.getDoubleTopic("Yaw").publish();
    odometryXPub = swerveTab.getDoubleTopic("Odometry X").publish();
    odometryYPub = swerveTab.getDoubleTopic("Odometry Y").publish();
    odometryYawPub = swerveTab.getDoubleTopic("Odometry Yaw").publish();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("gyro " + m_pigeon + " heading", getHeadingDegrees());
    SmartDashboard.putBoolean("Swerve Module Init Status", getModuleInitStatus());
    SmartDashboard.putNumber("Roll Offset", m_rollOffset);

    pitchPub.set(getPitchDegrees());
    rollPub.set(getRollDegrees() + getRollOffsetDegrees());
    yawPub.set(getHeadingDegrees());

    if (!m_limitCanUtil) {
      // Put not required stuff here
      odometryXPub.set(getOdometry().getEstimatedPosition().getX());
      odometryYPub.set(getOdometry().getEstimatedPosition().getY());
      odometryYawPub.set(getOdometry().getEstimatedPosition().getRotation().getDegrees());
    }
  }

  public void disabledPeriodic() {}

  @Override
  public void periodic() {
    if (DriverStation.isEnabled() && useHeadingTarget) {
      calculateRotationSpeed();
    }

    updateOdometry();
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed =
        SWERVE_DRIVE.kSwerveKinematics.toChassisSpeeds(
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    var currentTime = m_simTimer.get();
    double dt = currentTime - m_lastSimTime;

    m_simYaw += chassisSpeed.omegaRadiansPerSecond * dt;

    m_lastSimTime = currentTime;

    Unmanaged.feedEnable(20);
    m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }

  @Override
  public void close() throws Exception {
    for (var module : ModuleMap.orderedValuesList(m_swerveModules)) module.close();
  }
}
