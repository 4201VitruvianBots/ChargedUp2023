// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;
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
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SwerveDrive.SWERVE_MODULE_POSITION;
import frc.robot.utils.ModuleMap;
import java.util.HashMap;
import java.util.Map;

public class SwerveDrive extends SubsystemBase {

  private final HashMap<SWERVE_MODULE_POSITION, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
              SWERVE_MODULE_POSITION.FRONT_LEFT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.FRONT_LEFT,
                      new TalonFX(Constants.CAN.frontLeftTurnMotor),
                      new TalonFX(Constants.CAN.frontLeftDriveMotor),
                      new CANCoder(Constants.CAN.frontLeftCanCoder),
                      Constants.getInstance().SwerveDrive.frontLeftCANCoderOffset),
              SWERVE_MODULE_POSITION.FRONT_RIGHT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.FRONT_RIGHT,
                      new TalonFX(Constants.CAN.frontRightTurnMotor),
                      new TalonFX(Constants.CAN.frontRightDriveMotor),
                      new CANCoder(Constants.CAN.frontRightCanCoder),
                      Constants.getInstance().SwerveDrive.frontRightCANCoderOffset),
              SWERVE_MODULE_POSITION.BACK_LEFT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.BACK_LEFT,
                      new TalonFX(Constants.CAN.backLeftTurnMotor),
                      new TalonFX(Constants.CAN.backLeftDriveMotor),
                      new CANCoder(Constants.CAN.backLeftCanCoder),
                      Constants.getInstance().SwerveDrive.backLeftCANCoderOffset),
              SWERVE_MODULE_POSITION.BACK_RIGHT,
                  new SwerveModule(
                      SWERVE_MODULE_POSITION.BACK_RIGHT,
                      new TalonFX(Constants.CAN.backRightTurnMotor),
                      new TalonFX(Constants.CAN.backRightDriveMotor),
                      new CANCoder(Constants.CAN.backRightCanCoder),
                      Constants.getInstance().SwerveDrive.backRightCANCoderOffset)));

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.CAN.pigeon, "rio");
  private Trajectory m_trajectory;
  private boolean Initialize = false;

  private final SwerveDrivePoseEstimator m_odometry;
  private double m_simYaw;
  private DoublePublisher pitchPub, rollPub, yawPub, odometryXPub, odometryYPub, odometryYawPub;

  public boolean useHeadingTarget = false;
  private double m_desiredRobotHeading;

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(
          Constants.getInstance().SwerveDrive.kMaxRotationRadiansPerSecond,
          Constants.getInstance().SwerveDrive.kMaxRotationRadiansPerSecondSquared);
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private final ProfiledPIDController m_rotationController =
      new ProfiledPIDController(
          Constants.getInstance().SwerveDrive.kP_Rotation,
          Constants.getInstance().SwerveDrive.kI_Rotation,
          Constants.getInstance().SwerveDrive.kD_Rotation,
          m_constraints);
  private double m_rotationOutput;

  ChassisSpeeds chassisSpeeds;

  public SwerveDrive() {

    m_pigeon.configFactoryDefault();
    m_pigeon.setYaw(0);
    m_odometry =
        new SwerveDrivePoseEstimator(
            Constants.SwerveDrive.kSwerveKinematics,
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
    throttle *= Constants.getInstance().SwerveDrive.kMaxSpeedMetersPerSecond;
    strafe *= Constants.getInstance().SwerveDrive.kMaxSpeedMetersPerSecond;
    rotation *= Constants.getInstance().SwerveDrive.kMaxRotationRadiansPerSecond;

    if (useHeadingTarget) {
      // rotation = m_setpoint.velocity;
      rotation = m_rotationOutput;
      // SmartDashboard.putNumber("Rotation Target", Units.radiansToDegrees(m_setpoint.position));
      // SmartDashboard.putNumber("Rotation Speed ", Units.radiansToDegrees(rotation));
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
        ModuleMap.of(Constants.SwerveDrive.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]),
        Constants.getInstance().SwerveDrive.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
  }

  /*
   * Uses trapezoidal profile to set robot heading to a clear target
   */

  public void setRobotHeading(double desiredAngleSetpoint) {
    m_desiredRobotHeading = desiredAngleSetpoint;
  }

  public void calculateRotationSpeed() {
    // m_goal = new TrapezoidProfile.State(Units.degreesToRadians(m_desiredRobotHeading), 0);
    // var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    // m_setpoint = profile.calculate(0.02);

    m_rotationOutput =
        m_rotationController.calculate(getHeadingRotation2d().getRadians(), m_desiredRobotHeading);
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
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.getInstance().SwerveDrive.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
    var states = Constants.SwerveDrive.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
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
              Constants.SwerveDrive.kModuleTranslations.get(module.getModulePosition()),
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
    SmartDashboard.putBoolean("Swerve Module Init Status", Initialize);

    pitchPub.set(getPitchDegrees());
    rollPub.set(getRollDegrees());
    yawPub.set(getHeadingDegrees());
    odometryXPub.set(getOdometry().getEstimatedPosition().getX());
    odometryYPub.set(getOdometry().getEstimatedPosition().getY());
    odometryYawPub.set(getOdometry().getEstimatedPosition().getRotation().getDegrees());
  }

  public void disabledPeriodic() {}

  @Override
  public void periodic() {
    if (DriverStation.isEnabled() && useHeadingTarget) {
      calculateRotationSpeed();
    }

    Initialize = getModuleInitStatus();

    updateOdometry();
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed =
        Constants.SwerveDrive.kSwerveKinematics.toChassisSpeeds(
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }
}
