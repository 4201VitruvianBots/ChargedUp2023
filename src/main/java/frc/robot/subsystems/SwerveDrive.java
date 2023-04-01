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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_UTIL_LIMIT;
import frc.robot.Constants.SWERVEDRIVE.SWERVE_MODULE_POSITION;
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
                      new TalonFX(Constants.CAN.frontLeftTurnMotor),
                      new TalonFX(Constants.CAN.frontLeftDriveMotor),
                      new CANCoder(Constants.CAN.frontLeftCanCoder),
                      Constants.SWERVEDRIVE.frontLeftCANCoderOffset),
              SWERVE_MODULE_POSITION.FRONT_RIGHT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.FRONT_RIGHT,
                      new TalonFX(Constants.CAN.frontRightTurnMotor),
                      new TalonFX(Constants.CAN.frontRightDriveMotor),
                      new CANCoder(Constants.CAN.frontRightCanCoder),
                      Constants.SWERVEDRIVE.frontRightCANCoderOffset),
              SWERVE_MODULE_POSITION.BACK_LEFT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.BACK_LEFT,
                      new TalonFX(Constants.CAN.backLeftTurnMotor),
                      new TalonFX(Constants.CAN.backLeftDriveMotor),
                      new CANCoder(Constants.CAN.backLeftCanCoder),
                      Constants.SWERVEDRIVE.backLeftCANCoderOffset),
              SWERVE_MODULE_POSITION.BACK_RIGHT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.BACK_RIGHT,
                      new TalonFX(Constants.CAN.backRightTurnMotor),
                      new TalonFX(Constants.CAN.backRightDriveMotor),
                      new CANCoder(Constants.CAN.backRightCanCoder),
                      Constants.SWERVEDRIVE.backRightCANCoderOffset)));

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.CAN.pigeon, "rio");
  private double m_rollOffset;
  private Trajectory m_trajectory;

  private CAN_UTIL_LIMIT limitCanUtil = CAN_UTIL_LIMIT.NORMAL;

  private final SwerveDrivePoseEstimator m_odometry;
  private boolean m_simOverride = false;
  private double m_simYaw;
  private double m_simRoll;
  private DoublePublisher pitchPub, rollPub, yawPub, odometryXPub, odometryYPub, odometryYawPub;

  private boolean useHeadingTarget = false;
  private double m_desiredHeadingRadians;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(
          Constants.SWERVEDRIVE.kMaxRotationRadiansPerSecond,
          Constants.SWERVEDRIVE.kMaxRotationRadiansPerSecondSquared);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private final ProfiledPIDController m_rotationController =
      new ProfiledPIDController(
          Constants.SWERVEDRIVE.kP_Rotation,
          Constants.SWERVEDRIVE.kI_Rotation,
          Constants.SWERVEDRIVE.kD_Rotation,
          m_constraints);
  private double m_rotationOutput;

  ChassisSpeeds chassisSpeeds;
  private double m_maxVelocity = Constants.SWERVEDRIVE.kMaxSpeedMetersPerSecond;

  public SwerveDrive() {
    m_pigeon.configFactoryDefault();
    m_pigeon.setYaw(0);
    m_odometry =
        new SwerveDrivePoseEstimator(
            Constants.SWERVEDRIVE.kSwerveKinematics,
            getHeadingRotation2d(),
            getSwerveDriveModulePositionsArray(),
            new Pose2d());

    if (RobotBase.isReal()) {
      Timer.delay(1);
      resetModulesToAbsolute();
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
    rotation *= Constants.SWERVEDRIVE.kMaxRotationRadiansPerSecond;

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
        ModuleMap.of(Constants.SWERVEDRIVE.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

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
          m_rotationController.calculate(
              getHeadingRotation2d().getRadians(), m_desiredHeadingRadians);
    else m_rotationOutput = 0;
  }

  /*
   * ability to let head of swerve drive face the target
   */
  public void enableHeadingTarget(boolean enable) {
    useHeadingTarget = enable;
  }

  public void resetState() {
    m_setpoint =
        new TrapezoidProfile.State(
            Units.degreesToRadians(getHeadingDegrees()), Units.degreesToRadians(0));
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, m_maxVelocity);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public void setReduceCanUtilization(CAN_UTIL_LIMIT limitCan) {
    limitCanUtil = limitCan;
    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
      module.setReduceCanUtilization(limitCan);
    }
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
    var states = Constants.SWERVEDRIVE.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
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

  public Pigeon2 getPigeon() {
    return m_pigeon;
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

  public void setNeutralMode(NeutralMode mode) {
    for (SwerveModule module : m_swerveModules.values()) {
      module.setDriveNeutralMode(mode);
      module.setTurnNeutralMode(mode);
    }
  }

  public void setMaxVelocity(double mps) {
    m_maxVelocity = mps;
  }

  public void setCurrentTrajectory(Trajectory trajectory) {
    m_trajectory = trajectory;
  }

  public Trajectory getCurrentTrajectory() {
    return m_trajectory;
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
              Constants.SWERVEDRIVE.kModuleTranslations.get(module.getModulePosition()),
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

  private void updateSmartDashboard(CAN_UTIL_LIMIT limitCan) {
    SmartDashboard.putNumber("gyro " + m_pigeon + " heading", getHeadingDegrees());
    SmartDashboard.putBoolean("Swerve Module Init Status", getModuleInitStatus());
    SmartDashboard.putNumber("Roll Offset", m_rollOffset);

    switch (limitCan) {
      case NORMAL:
        // Put not required stuff here
        pitchPub.set(getPitchDegrees());
        rollPub.set(getRollDegrees() + getRollOffsetDegrees());
        yawPub.set(getHeadingDegrees());
        odometryXPub.set(getOdometry().getEstimatedPosition().getX());
        odometryYPub.set(getOdometry().getEstimatedPosition().getY());
        odometryYawPub.set(getOdometry().getEstimatedPosition().getRotation().getDegrees());
        break;
      default:
      case LIMITED:
        pitchPub.set(getPitchDegrees());
        rollPub.set(getRollDegrees() + getRollOffsetDegrees());
        yawPub.set(getHeadingDegrees());
        break;
    }
  }

  public void disabledPeriodic() {}

  @Override
  public void periodic() {
    if (DriverStation.isEnabled() && useHeadingTarget) {
      calculateRotationSpeed();
    }

    updateOdometry();
    updateSmartDashboard(limitCanUtil);
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed =
        Constants.SWERVEDRIVE.kSwerveKinematics.toChassisSpeeds(
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }

  @Override
  public void close() throws Exception {

    for (var module : ModuleMap.orderedValuesList(m_swerveModules)) module.close();
  }
}
