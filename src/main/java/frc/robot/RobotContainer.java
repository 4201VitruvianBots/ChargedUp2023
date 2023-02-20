// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.RunReverseIntake;
import frc.robot.commands.Intake.RunWristJoystick;
import frc.robot.commands.auto.*;
import frc.robot.commands.elevator.IncrementElevatorHeight;
import frc.robot.commands.elevator.MoveToElevatorHeight;
import frc.robot.commands.elevator.SetElevatorControlLoop;
import frc.robot.commands.led.SetPieceTypeIntent;
import frc.robot.commands.swerve.ResetOdometry;
import frc.robot.commands.swerve.SetSwerveCoastMode;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveDriveBalance;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.USB;
import frc.robot.constants.ConstantsGridLock;
import frc.robot.constants.ConstantsRushHour;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.MemoryLog;
import frc.robot.subsystems.Controls;
// import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.elevatorHeights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.PieceType;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import java.util.HashMap;
import java.util.Objects;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DataLog m_logger = DataLogManager.getLog();

  // The robot's subsystems and commands are defined here...4
  private final Intake m_intake = new Intake();
  private final Elevator m_elevator = new Elevator();
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final Controls m_controls = new Controls();
  private final Vision m_vision = new Vision(m_swerveDrive, m_logger, m_controls);
  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive, m_vision, m_elevator);
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final Wrist m_wrist = new Wrist();
  private final LED m_led = new LED(m_controls);
  private final StateHandler m_stateHandler =
      new StateHandler(m_intake, m_wrist, m_swerveDrive, m_fieldSim, m_elevator, m_led, m_vision);
  // private final DistanceSensor m_distanceSensor = new DistanceSensor();

  HashMap<String, Command> m_eventMap = new HashMap<>();
  private SwerveAutoBuilder m_autoBuilder;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final MemoryLog m_memorylog = new MemoryLog();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  static Joystick leftJoystick = new Joystick(USB.leftJoystick);

  static Joystick rightJoystick = new Joystick(USB.rightJoystick);
  public final CommandXboxController xboxController = new CommandXboxController(USB.xBoxController);

  public Trigger[] leftJoystickTriggers = new Trigger[2];
  public Trigger[] rightJoystickTriggers = new Trigger[2];

  public void initializeSubsystems() {
    m_swerveDrive.setDefaultCommand(
        new SetSwerveDrive(
            m_swerveDrive,
            () -> leftJoystick.getRawAxis(1),
            () -> leftJoystick.getRawAxis(0),
            () -> rightJoystick.getRawAxis(0)));

    // Control elevator height by moving the joystick up and down
    m_elevator.setDefaultCommand(new IncrementElevatorHeight(m_elevator, xboxController::getLeftY));
    m_fieldSim.initSim();
    m_wrist.setDefaultCommand(new RunWristJoystick(m_wrist, xboxController::getRightX));
  }

  public RobotContainer() {
    initializeSubsystems();
    initAutoBuilder();
    initializeAutoChooser();

    // Configure the button bindings
    configureBindings();

    // Choose which constants class to use
    chooseConstants();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() { // TODO: Replace Joystick Button?
    for (int i = 0; i < leftJoystickTriggers.length; i++)
      leftJoystickTriggers[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightJoystickTriggers.length; i++)
      rightJoystickTriggers[i] = new JoystickButton(rightJoystick, (i + 1));

    xboxController.leftBumper().onTrue(new SetPieceTypeIntent(m_led, PieceType.CONE));
    xboxController.rightBumper().onTrue(new SetPieceTypeIntent(m_led, PieceType.CONE));

    xboxController.leftTrigger().whileTrue(new RunIntake(m_intake, 0.5));
    xboxController.rightTrigger().whileTrue(new RunReverseIntake(m_intake, 0.5));

    // Elevator button bindings
    xboxController.a().whileTrue(new MoveToElevatorHeight(m_elevator, elevatorHeights.LOW));
    xboxController.b().whileTrue(new MoveToElevatorHeight(m_elevator, elevatorHeights.MID));
    xboxController.x().whileTrue(new MoveToElevatorHeight(m_elevator, elevatorHeights.STOWED));
    xboxController.y().whileTrue(new MoveToElevatorHeight(m_elevator, elevatorHeights.HIGH));

    // Will switch between closed and open loop on button press
    xboxController.start().onTrue(new SetElevatorControlLoop(m_elevator));

    leftJoystickTriggers[0].whileTrue(
        new SetSwerveDriveBalance(m_swerveDrive, null, null, null)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    SmartDashboard.putData(new ResetOdometry(m_swerveDrive));
    SmartDashboard.putData(new SetSwerveCoastMode(m_swerveDrive));
  }

  public void disableInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Coast);
    m_swerveDrive.disabledInit();
  }

  public void teleopInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Brake);
  }

  private void initAutoBuilder() {
    m_eventMap.put("wait", new WaitCommand(5));

    m_autoBuilder =
        new SwerveAutoBuilder(
            m_swerveDrive::getPoseMeters,
            m_swerveDrive::setOdometry,
            Constants.constants.SwerveDrive.kSwerveKinematics,
            new PIDConstants(
                Constants.constants.SwerveDrive.kP_Translation,
                Constants.constants.SwerveDrive.kI_Translation,
                Constants.constants.SwerveDrive.kD_Translation),
            new PIDConstants(
                Constants.constants.SwerveDrive.kP_Rotation,
                Constants.constants.SwerveDrive.kI_Rotation,
                Constants.constants.SwerveDrive.kD_Rotation),
            m_swerveDrive::setSwerveModuleStatesAuto,
            m_eventMap,
            false,
            m_swerveDrive);
  }
  /** Use this to pass the autonomous command to the main {@link Robot} class. */
  public void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    //   m_autoChooser.addOption("MiddleOneConeBalance", new
    // RedMiddleOneConeBalance(m_swerveDrive, m_fieldSim));

    // m_autoChooser.addOption("DriveSideway2", new DriveSideway2(m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption(
        "BlueTopConeCubeBalance",
        new BlueTopConeCubeBalance(m_autoBuilder, m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption(
        "RedTopTwoConeBalance", new RedTopTwoConeBalance(m_autoBuilder, m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption(
        "DriveSideway", new DriveSideway(m_autoBuilder, m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption(
        "DriveForward", new DriveForward(m_autoBuilder, m_swerveDrive, m_fieldSim));
    // m_autoChooser.addOption(
    //     "DriveForwardIntakeTest", new DriveForwardIntakeTest(m_swerveDrive, m_fieldSim));
    m_autoChooser.addOption("WayPoint", new Waypoint(m_autoBuilder, m_swerveDrive, m_fieldSim));

    m_autoChooser.addOption(
        "BlueMiddleTwoConeBalance",
        new BlueMiddleTwoConeBottomBalance(m_autoBuilder, m_swerveDrive, m_fieldSim));
    // m_autoChooser.addOption("DriveTest", new DriveTest(m_swerveDrive, m_fieldSim));
    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  // Switches between constants class depending on the MAC address of the roboRIO we're running on
  public void chooseConstants() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    String mac = inst.getTable("RIO-Info").getEntry("MAC").getString("N/A");
    if (Objects.equals(mac, Constants.alphaRobotMAC)) {
      Constants.constants = new ConstantsRushHour();
    } else if (Objects.equals(mac, Constants.betaRobotMAC)) {
      Constants.constants = new ConstantsGridLock();
    }
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }

  public void simulationPeriodic() {
    m_elevator.simulationPeriodic();
    m_memorylog.simulationPeriodic();
  }

  public void periodic() {
    m_fieldSim.periodic();
    // Rumbles the controller if the robot is on target based off FieldSim
    xboxController.getHID().setRumble(RumbleType.kBothRumble, m_stateHandler.isOnTarget ? 1 : 0);
  }
}
