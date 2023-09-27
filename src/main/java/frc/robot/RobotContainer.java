// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.USB;
// import frc.robot.commands.auto.BumpOnePickUp;
// import frc.robot.commands.auto.CenterOneBalance;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.LimitSwerveJoystickInput;
import frc.robot.commands.swerve.ResetOdometry;
import frc.robot.commands.swerve.SetRollOffset;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.subsystems.*;
import frc.robot.utils.LogManager;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements AutoCloseable {
  private final DataLog m_logger = DataLogManager.getLog();

  // Initialize used utils
  private final LogManager m_logManager = new LogManager();
  //  private final DistanceSensor m_distanceSensor = new DistanceSensor();

  // The robot's subsystems and commands are defined here...
  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  private SendableChooser<List<PathPlannerTrajectory>> autoPlotter;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);

  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController xboxController =
      new CommandXboxController(USB.xBoxController);

  private final Trigger[] leftJoystickTriggers = new Trigger[2]; // left joystick buttons
  private final Trigger[] rightJoystickTriggers = new Trigger[2]; // right joystick buttons

  public RobotContainer() {
    initializeSubsystems();
    m_logger.pause();
    configureBindings();

    initializeAutoChooser();
  }

  public void initializeSubsystems() {
    m_swerveDrive.setDefaultCommand(
        new SetSwerveDrive(
            m_swerveDrive,
            () -> leftJoystick.getRawAxis(1),
            () -> leftJoystick.getRawAxis(0),
            () -> rightJoystick.getRawAxis(0)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    for (int i = 0; i < leftJoystickTriggers.length; i++)
      leftJoystickTriggers[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightJoystickTriggers.length; i++)
      rightJoystickTriggers[i] = new JoystickButton(rightJoystick, (i + 1));

    leftJoystickTriggers[0].whileTrue(new AutoBalance(m_swerveDrive));

    rightJoystickTriggers[0].whileTrue(new LimitSwerveJoystickInput(m_swerveDrive));

    // Add Smartdashboard Buttons
    SmartDashboard.putData(new ResetOdometry(m_swerveDrive));
    SmartDashboard.putData(new SetSwerveNeutralMode(m_swerveDrive, NeutralMode.Coast));
    SmartDashboard.putData(new SetRollOffset(m_swerveDrive));
  }

  public void disableInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Coast);
  }

  public void teleopInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Brake);
  }

  public void autonomousInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Brake);
  }

  /** Use this to pass the autonomous command to the main {@link Robot} class. */
  public void initializeAutoChooser() {}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public SwerveDrive getSwerveDrive() {
    return m_swerveDrive;
  }

  public Joystick getLeftJoystick() {
    return leftJoystick;
  }

  //  public DistanceSensor getDistanceSensor() {
  //    return m_distanceSensor;
  //  }

  public void periodic() {}

  public void disabledPeriodic() {}

  public void testPeriodic() {}

  public void simulationPeriodic() {}

  @Override
  public void close() throws Exception {
    //    m_distanceSensor.close();
    m_logger.close();
  }
}
