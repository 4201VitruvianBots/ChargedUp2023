// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SwerveDriveModulePosition;
import frc.robot.utils.ModuleMap;
import java.util.HashMap;
import java.util.Map;

public class SwerveDrive extends SubsystemBase {

  private final HashMap<SwerveDriveModulePosition, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
              SwerveDriveModulePosition.FRONT_LEFT,
                  new SwerveModule(
                      SwerveDriveModulePosition.FRONT_LEFT,
                      new TalonFX(Constants.CAN.frontLeftTurnMotor),
                      new TalonFX(Constants.CAN.frontLeftDriveMotor),
                      new CANCoder(Constants.CAN.frontLeftCanCoder),
                      Constants.constants.SwerveDrive.frontLeftCANCoderOffset),
              SwerveDriveModulePosition.FRONT_RIGHT,
                  new SwerveModule(
                      SwerveDriveModulePosition.FRONT_RIGHT,
                      new TalonFX(Constants.CAN.frontRightTurnMotor),
                      new TalonFX(Constants.CAN.frontRightDriveMotor),
                      new CANCoder(Constants.CAN.frontRightCanCoder),
                      Constants.constants.SwerveDrive.frontRightCANCoderOffset),
              SwerveDriveModulePosition.BACK_LEFT,
                  new SwerveModule(
                      SwerveDriveModulePosition.BACK_LEFT,
                      new TalonFX(Constants.CAN.backLeftTurnMotor),
                      new TalonFX(Constants.CAN.backLeftDriveMotor),
                      new CANCoder(Constants.CAN.backLeftCanCoder),
                      Constants.constants.SwerveDrive.backLeftCANCoderOffset),
              SwerveDriveModulePosition.BACK_RIGHT,
                  new SwerveModule(
                      SwerveDriveModulePosition.BACK_RIGHT,
                      new TalonFX(Constants.CAN.backRightTurnMotor),
                      new TalonFX(Constants.CAN.backRightDriveMotor),
                      new CANCoder(Constants.CAN.backRightCanCoder),
                      Constants.constants.SwerveDrive.backRightCANCoderOffset)));

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.CAN.pigeon, "rio");
  private Trajectory m_trajectory;
  private boolean Initialize = false;

  private final SwerveDrivePoseEstimator m_odometry;
  private double m_simYaw;
  private DoublePublisher swervePitch, swerveRoll, swerveYaw;

  public SwerveDrive() {
    m_pigeon.configFactoryDefault();
    m_pigeon.setYaw(0);
    m_odometry =
        new SwerveDrivePoseEstimator(
            Constants.constants.SwerveDrive.kSwerveKinematics,
            getHeadingRotation2d(),
            getSwerveDriveModulePositionsArray(),
            new Pose2d());

    Timer.delay(1);
    if (RobotBase.isReal()) resetModulesToAbsolute();
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
    throttle *= Constants.constants.SwerveDrive.kMaxSpeedMetersPerSecond;
    strafe *= Constants.constants.SwerveDrive.kMaxSpeedMetersPerSecond;
    rotation *= Constants.constants.SwerveDrive.kMaxRotationRadiansPerSecond;

    ChassisSpeeds chassisSpeeds =
        isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                throttle, strafe, rotation, getHeadingRotation2d())
            : new ChassisSpeeds(throttle, strafe, rotation);

    Map<SwerveDriveModulePosition, SwerveModuleState> moduleStates =
        ModuleMap.of(
            Constants.constants.SwerveDrive.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]),
        Constants.constants.SwerveDrive.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.constants.SwerveDrive.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
    var states =
        Constants.constants.SwerveDrive.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    setSwerveModuleStates(states, false);
  }

  public void setOdometry(Pose2d pose) {
    m_pigeon.setYaw(pose.getRotation().getDegrees());
    m_odometry.resetPosition(getHeadingRotation2d(), getSwerveDriveModulePositionsArray(), pose);
  }

  public double getPitchDegrees() {
    return m_pigeon.getPitch();
  }

  public double getRollDegrees() {
    return m_pigeon.getRoll();
  }

  public double getHeadingDegrees() {
    return m_pigeon.getYaw();
    // return 0;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getEstimatedPosition();
  }

  public SwerveModule getSwerveModule(SwerveDriveModulePosition modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public Map<SwerveDriveModulePosition, SwerveModuleState> getModuleStates() {
    Map<SwerveDriveModulePosition, SwerveModuleState> map = new HashMap<>();
    for (SwerveDriveModulePosition i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getState());
    return map;
  }

  public Map<SwerveDriveModulePosition, SwerveModulePosition> getModulePositions() {
    Map<SwerveDriveModulePosition, SwerveModulePosition> map = new HashMap<>();
    for (SwerveDriveModulePosition i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getPosition());
    return map;
  }

  public SwerveModulePosition[] getSwerveDriveModulePositionsArray() {
    return ModuleMap.orderedValues(getModulePositions(), new SwerveModulePosition[0]);
  }

  public Map<SwerveDriveModulePosition, Pose2d> getModulePoses() {
    Map<SwerveDriveModulePosition, Pose2d> map = new HashMap<>();
    for (SwerveDriveModulePosition i : m_swerveModules.keySet())
      map.put(i, m_swerveModules.get(i).getModulePose());
    return map;
  }

  public boolean getModuleInitStatus() {
    for (SwerveDriveModulePosition i : m_swerveModules.keySet()) {

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

    //    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
    //      Translation2d modulePositionFromChassis = getPoseMeters().getTranslation()
    //              .rotateBy(getHeadingRotation2d())
    //              .plus(kModuleTranslations.get(module.getModulePosition()));
    //
    //      module.setModulePose(
    //          new Pose2d(
    //              modulePositionFromChassis,
    //              getHeadingRotation2d().plus(module.getHeadingRotation2d())));
    //    }

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
      Translation2d modulePositionFromChassis =
              Constants.constants.SwerveDrive.kModuleTranslations
              .get(module.getModulePosition())
              .rotateBy(getHeadingRotation2d())
              .plus(getPoseMeters().getTranslation());
      module.setModulePose(
          new Pose2d(
              modulePositionFromChassis,
              module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }

  private void initSmartDashboard() {
    SmartDashboard.putData(this);

    var swerveTab = NetworkTableInstance.getDefault().getTable("Swerve");
    swervePitch = swerveTab.getDoubleTopic("Pitch").publish();
    swerveRoll = swerveTab.getDoubleTopic("Roll").publish();
    swerveYaw = swerveTab.getDoubleTopic("Yaw").publish();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("gyro " + m_pigeon + " heading", getHeadingDegrees());
    SmartDashboard.putBoolean("ModuleInitStatus", Initialize);
    SmartDashboard.putNumber("X Odometry", m_odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Y Odometry", m_odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber(
        "Rotation Odometry", m_odometry.getEstimatedPosition().getRotation().getDegrees());

    swervePitch.set(getPitchDegrees());
    swerveRoll.set(getRollDegrees());
    swerveYaw.set(getHeadingDegrees());
  }

  @Override
  public void periodic() {
    if (Initialize == false) {
      if (getModuleInitStatus()) {
        Initialize = true;
      }
    }
    updateOdometry();
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed =
        Constants.constants.SwerveDrive.kSwerveKinematics.toChassisSpeeds(
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }
}
