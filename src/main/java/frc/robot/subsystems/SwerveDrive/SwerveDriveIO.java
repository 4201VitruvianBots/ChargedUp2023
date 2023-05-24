// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import static frc.robot.subsystems.StateHandler.m_chassisRoot2d;

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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.SWERVE_DRIVE.SWERVE_MODULE_POSITION;
import frc.robot.subsystems.StateHandler;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.ModuleMap;
import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

public class SwerveDriveIO extends SubsystemBase implements AutoCloseable {

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

                  private final HashMap<SWERVE_MODULE_POSITION, ModuleIOInputsAutoLogged> m_swerveModuleInputs =
                  new HashMap<>(
                      Map.of(
                          SWERVE_MODULE_POSITION.FRONT_LEFT,
                          new ModuleIOInputsAutoLogged(),
                          SWERVE_MODULE_POSITION.FRONT_RIGHT,
                          new ModuleIOInputsAutoLogged(),
                          SWERVE_MODULE_POSITION.BACK_LEFT,
                          new ModuleIOInputsAutoLogged(),
                          SWERVE_MODULE_POSITION.BACK_RIGHT,
                          new ModuleIOInputsAutoLogged()));
                
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs =
  new GyroIOInputsAutoLogged();
  

  private final LoggedTunableNumber SwerveKp_X = 
  new LoggedTunableNumber("Swerve/SwerveKp_X");
 
  private final LoggedTunableNumber SwerveKd_X = 
  new LoggedTunableNumber("Swerve/SwerveKp_X");

  private final LoggedTunableNumber SwerveKp_Y = 
  new LoggedTunableNumber("Swerve/SwerveKp_Y");

  private final LoggedTunableNumber SwerveKd_Y = 
  new LoggedTunableNumber("Swerve/SwerveKd_Y");
  
  private final LoggedTunableNumber SwerveKp_Turn = 
  new LoggedTunableNumber("Swerve/SwerveKp_Turn");
  


 
 
 

  public SwerveDriveIO(GyroIO gyroIO) {
    this.gyroIO = gyroIO;

    initSmartDashboard();


    
   
  }


    /** Setting field vs Robot Relative */
 







  public void updateOdometry() {

  }

  private void initSmartDashboard() {
    SmartDashboard.putData(this);

  }

  private void updateSmartDashboard() {

  }

  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
    gyroIO.updateInputs(gyroInputs);
    for (SWERVE_MODULE_POSITION i : SWERVE_MODULE_POSITION.values()) {
      
    }
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);


  }

  @Override
  public void simulationPeriodic() {

  }

  @Override
  public void close() throws Exception {
  }
}
