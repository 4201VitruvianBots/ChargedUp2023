// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.STATEHANDLER.INTAKING_STATES;
import frc.robot.Constants.STATEHANDLER.SUPERSTRUCTURE_STATE;
import frc.robot.Constants.USB;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.*;
import frc.robot.commands.auto.*;
// import frc.robot.commands.auto.RedTopTwoBalance;
import frc.robot.commands.elevator.*;
import frc.robot.commands.led.GetSubsystemStates;
import frc.robot.commands.led.SetPieceTypeIntent;
import frc.robot.commands.sim.fieldsim.SwitchTargetNode;
import frc.robot.commands.swerve.ResetOdometry;
import frc.robot.commands.swerve.SetSwerveCoastMode;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveMaxTranslationVeolcity;
import frc.robot.commands.wrist.*;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.MemoryLog;
import frc.robot.subsystems.*;
import frc.robot.utils.DistanceSensor;
import frc.robot.utils.LogManager;
import frc.robot.utils.TrajectoryUtils;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements AutoCloseable {
  private final DataLog m_logger = DataLogManager.getLog();

  // The robot's subsystems and commands are defined here...
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();
  private final Wrist m_wrist = new Wrist(m_intake);
  private final Controls m_controls = new Controls();
  private final Vision m_vision = new Vision(m_swerveDrive, m_logger, m_controls, m_intake);
  private final FieldSim m_fieldSim =
      new FieldSim(m_swerveDrive, m_vision, m_elevator, m_wrist, m_controls);
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final LEDSubsystem m_led = new LEDSubsystem(m_controls);
  private SendableChooser<List<PathPlannerTrajectory>> autoPlotter;

  private final StateHandler m_stateHandler =
      new StateHandler(m_intake, m_wrist, m_swerveDrive, m_fieldSim, m_elevator, m_led, m_vision);

  //  private final DistanceSensor m_distanceSensor = new DistanceSensor();
  // private final DistanceSensor m_distanceSensor = new DistanceSensor();

  HashMap<String, Command> m_eventMap = new HashMap<>();
  private SwerveAutoBuilder m_autoBuilder;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  // Initialize used utils
  private final MemoryLog m_memorylog = new MemoryLog();
  private final LogManager m_logManager = new LogManager();
  private final DistanceSensor m_distanceSensor = new DistanceSensor();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);

  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  public CommandXboxController xboxController = new CommandXboxController(USB.xBoxController);

  public Trigger[] leftJoystickTriggers = new Trigger[2]; // left joystick buttons
  public Trigger[] rightJoystickTriggers = new Trigger[2]; // right joystick buttons

  public RobotContainer() {
    initializeSubsystems();
    m_logger.pause();
    configureBindings();

    initAutoBuilder();
    initializeAutoChooser();
  }

  public void initializeSubsystems() {
    m_swerveDrive.setDefaultCommand(
        new SetSwerveDrive(
            m_swerveDrive,
            () -> leftJoystick.getRawAxis(1),
            () -> leftJoystick.getRawAxis(0),
            () -> rightJoystick.getRawAxis(0)));

    // Control elevator height by moving the joystick up and down
    m_elevator.setDefaultCommand(new IncrementElevatorHeight(m_elevator, xboxController::getLeftY));
    m_wrist.setDefaultCommand(new RunWristJoystick(m_wrist, xboxController::getRightY));
    m_led.setDefaultCommand(new GetSubsystemStates(m_led, m_controls, m_stateHandler, m_intake));

    SmartDashboard.putData(new ResetElevatorHeightMeters(m_elevator, 0));
    SmartDashboard.putData(new ResetAngleDegrees(m_wrist, -15.0));
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

    leftJoystickTriggers[0].whileTrue(
        new SetSwerveMaxTranslationVeolcity(
            m_swerveDrive, Constants.SWERVEDRIVE.kMaxSpeedMetersPerSecond / 25.0));

    leftJoystickTriggers[1].whileTrue(
        new IntakeVisionAlignment(
            m_vision,
            m_swerveDrive,
            () -> leftJoystick.getRawAxis(1),
            () -> leftJoystick.getRawAxis(0),
            () -> rightJoystick.getRawAxis(0)));

    xboxController.leftTrigger(0.1).whileTrue(new RunIntakeCone(m_intake, 0.9));
    xboxController.rightTrigger(0.1).whileTrue(new RunIntakeCube(m_intake, 0.74));

    // Score button Bindings

    // Score LOW Setpoints
    xboxController
        .a()
        .whileTrue(
            new ConditionalCommand(
                new SetElevatorDesiredSetpoint(
                    m_elevator, ELEVATOR.SETPOINT.SCORE_LOW_CONE.get(), xboxController::getLeftY),
                new SetElevatorDesiredSetpoint(
                    m_elevator, ELEVATOR.SETPOINT.SCORE_LOW_CUBE.get(), xboxController::getLeftY),
                () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));
    xboxController
        .a()
        .whileTrue(
            new ConditionalCommand(
                new SetWristDesiredSetpoint(
                    m_wrist, WRIST.SETPOINT.SCORE_LOW_REVERSE.get(), xboxController::getRightY),
                new SetWristDesiredSetpoint(
                    m_wrist, WRIST.SETPOINT.SCORE_LOW_REVERSE.get(), xboxController::getRightY),
                () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));

    // Score MID Setpoints
    xboxController
        .b()
        .whileTrue(
            new ConditionalCommand(
                new SetElevatorDesiredSetpoint(
                    m_elevator, ELEVATOR.SETPOINT.SCORE_MID_CONE.get(), xboxController::getLeftY),
                new SetElevatorDesiredSetpoint(
                    m_elevator, ELEVATOR.SETPOINT.SCORE_MID_CUBE.get(), xboxController::getLeftY),
                () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));
    xboxController
        .b()
        .whileTrue(
            new ConditionalCommand(
                new SetWristDesiredSetpoint(
                    m_wrist, WRIST.SETPOINT.SCORE_MID_CONE.get(), xboxController::getRightY),
                new SetWristDesiredSetpoint(
                    m_wrist, WRIST.SETPOINT.SCORE_MID_CUBE.get(), xboxController::getRightY),
                () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));

    // Stowed
    xboxController
        .x()
        .whileTrue(
            new SetElevatorDesiredSetpoint(
                m_elevator, ELEVATOR.SETPOINT.STOWED.get(), xboxController::getLeftY));
    xboxController
        .x()
        .whileTrue(
            new SetWristDesiredSetpoint(
                m_wrist, WRIST.SETPOINT.STOWED.get(), xboxController::getRightY));

    // High
    xboxController
        .y()
        .whileTrue(
            new ConditionalCommand(
                new SetElevatorDesiredSetpoint(
                    m_elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get(), xboxController::getLeftY),
                new SetElevatorDesiredSetpoint(
                    m_elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CUBE.get(), xboxController::getLeftY),
                () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));
    xboxController
        .y()
        .whileTrue(
            new ConditionalCommand(
                new SetWristDesiredSetpoint(
                    m_wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get(), xboxController::getRightY),
                new SetWristDesiredSetpoint(
                    m_wrist, WRIST.SETPOINT.SCORE_HIGH_CUBE.get(), xboxController::getRightY),
                () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));
    // Toggle elevator, wrist control state
    xboxController
        .povUp()
        .onTrue(
            new SetElevatorDesiredSetpoint(m_elevator, ELEVATOR.SETPOINT.INTAKING_EXTENDED.get()));
    xboxController
        .povUp()
        .onTrue(new SetWristDesiredSetpoint(m_wrist, WRIST.SETPOINT.INTAKING_EXTENDED.get()));

    // Will switch between closed and open loop on button press
    xboxController.back().onTrue(new ToggleElevatorControlMode(m_elevator));
    xboxController.start().onTrue(new ToggleWristControlMode(m_wrist));
    xboxController.rightBumper().whileTrue(new SetPieceTypeIntent(m_led, INTAKING_STATES.CONE));
    xboxController
        .rightBumper()
        .whileTrue(
            new SetWristDesiredSetpoint(
                m_wrist, WRIST.SETPOINT.INTAKING_LOW.get(), xboxController::getRightY));
    xboxController.leftBumper().whileTrue(new SetPieceTypeIntent(m_led, INTAKING_STATES.CUBE));
    xboxController
        .leftBumper()
        .whileTrue(
            new SetWristDesiredSetpoint(
                m_wrist, WRIST.SETPOINT.INTAKING_LOW.get(), xboxController::getRightY));

    // Will switch our target node on the field sim to the adjacent node on D-pad press
    xboxController.povLeft().onTrue(new SwitchTargetNode(m_stateHandler, true));
    xboxController.povRight().onTrue(new SwitchTargetNode(m_stateHandler, false));

    SmartDashboard.putData(new ResetOdometry(m_swerveDrive));
    SmartDashboard.putData(new SetSwerveCoastMode(m_swerveDrive));

    initTestController();
  }

  private void initTestController() {
    if (RobotBase.isSimulation()) {
      CommandPS4Controller testController = new CommandPS4Controller(3);

      testController.axisGreaterThan(3, 0.1).whileTrue(new RunIntakeCone(m_intake, 0.64));
      testController
          .axisGreaterThan(3, 0.1)
          .whileTrue(
              new ConditionalCommand(
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.INTAKING_LOW.get(), testController::getRightY),
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get(), testController::getRightY),
                  () ->
                      m_stateHandler.getCurrentZone().getZone()
                          == SUPERSTRUCTURE_STATE.LOW_ZONE.getZone()));

      testController.axisGreaterThan(4, 0.1).whileTrue(new RunIntakeCube(m_intake, 0.64));
      testController
          .axisGreaterThan(4, 0.1)
          .whileTrue(
              new ConditionalCommand(
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.INTAKING_LOW.get(), testController::getRightY),
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get(), testController::getRightY),
                  () ->
                      m_stateHandler.getCurrentZone().getZone()
                          == SUPERSTRUCTURE_STATE.LOW_ZONE.getZone()));

      // Score button Bindings

      // Score LOW Setpoints
      testController
          .cross()
          .whileTrue(
              new ConditionalCommand(
                  new SetElevatorDesiredSetpoint(
                      m_elevator, ELEVATOR.SETPOINT.SCORE_LOW_CONE.get(), testController::getLeftY),
                  new SetElevatorDesiredSetpoint(
                      m_elevator, ELEVATOR.SETPOINT.SCORE_LOW_CUBE.get(), testController::getLeftY),
                  () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));
      testController
          .cross()
          .whileTrue(
              new ConditionalCommand(
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.SCORE_LOW_CONE.get(), testController::getRightY),
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.SCORE_LOW_CUBE.get(), testController::getRightY),
                  () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));

      // Score MID Setpoints
      testController
          .circle()
          .whileTrue(
              new ConditionalCommand(
                  new SetElevatorDesiredSetpoint(
                      m_elevator, ELEVATOR.SETPOINT.SCORE_MID_CONE.get(), testController::getLeftY),
                  new SetElevatorDesiredSetpoint(
                      m_elevator, ELEVATOR.SETPOINT.SCORE_MID_CUBE.get(), testController::getLeftY),
                  () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));
      testController
          .circle()
          .whileTrue(
              new ConditionalCommand(
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.SCORE_MID_CONE.get(), testController::getRightY),
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.SCORE_MID_CUBE.get(), testController::getRightY),
                  () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));

      // Stowed
      testController
          .square()
          .whileTrue(
              new SetElevatorDesiredSetpoint(
                  m_elevator, ELEVATOR.SETPOINT.STOWED.get(), testController::getLeftY));
      testController
          .square()
          .whileTrue(
              new SetWristDesiredSetpoint(
                  m_wrist, WRIST.SETPOINT.STOWED.get(), testController::getRightY));

      // High
      testController
          .triangle()
          .whileTrue(
              new ConditionalCommand(
                  new SetElevatorDesiredSetpoint(
                      m_elevator,
                      ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get(),
                      testController::getLeftY),
                  new SetElevatorDesiredSetpoint(
                      m_elevator,
                      ELEVATOR.SETPOINT.SCORE_HIGH_CUBE.get(),
                      testController::getLeftY),
                  () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));
      testController
          .triangle()
          .whileTrue(
              new ConditionalCommand(
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get(), testController::getRightY),
                  new SetWristDesiredSetpoint(
                      m_wrist, WRIST.SETPOINT.SCORE_HIGH_CUBE.get(), testController::getRightY),
                  () -> m_intake.getHeldGamepiece() == Constants.INTAKE.HELD_GAMEPIECE.CONE));

      // Toggle elevator, wrist control state
      testController
          .povDown()
          .onTrue(new SetElevatorDesiredSetpoint(m_elevator, ELEVATOR.SETPOINT.STOWED.get()));
      testController
          .povDown()
          .onTrue(new SetWristDesiredSetpoint(m_wrist, WRIST.SETPOINT.STOWED.get()));

      // Will switch between closed and open loop on button press
      testController.share().onTrue(new ToggleElevatorControlMode(m_elevator));
      testController.options().onTrue(new ToggleWristControlMode(m_wrist));
    }
  }

  public void disableInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Coast);
  }

  public void teleopInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Brake);
    m_elevator.setDesiredPositionMeters(m_elevator.getHeightMeters());
    m_elevator.resetState();
    m_wrist.setDesiredPositionRadians(m_wrist.getPositionRadians());
    m_wrist.resetState();
    m_swerveDrive.resetState();
    m_stateHandler.init();
  }

  public void autonomousInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Brake);
    m_elevator.setDesiredPositionMeters(m_elevator.getHeightMeters());
    m_elevator.resetState();
    m_wrist.setDesiredPositionRadians(m_wrist.getPositionRadians());
    m_wrist.resetState();
    m_swerveDrive.resetState();
    m_stateHandler.init();
  }

  private void initAutoBuilder() {
    m_eventMap.put("wait1", new WaitCommand(0.5));
    m_eventMap.put("RunIntakeCone", new AutoRunIntakeCone(m_intake, 0.9, m_vision, m_swerveDrive));
    m_eventMap.put("RunIntakeCube", new AutoRunIntakeCube(m_intake, 0.5, m_vision, m_swerveDrive));
    m_eventMap.put(
        "RunIntakeConeReverse", new AutoRunIntakeCube(m_intake, -0.5, m_vision, m_swerveDrive));
    m_eventMap.put(
        "RunIntakeCubeReverse", new AutoRunIntakeCone(m_intake, -0.5, m_vision, m_swerveDrive));
    m_eventMap.put("IntakeHoldCone", new AutoRunIntakeCone(m_intake, 0.2, m_vision, m_swerveDrive));
    m_eventMap.put("IntakeHoldCube", new AutoRunIntakeCube(m_intake, 0.2, m_vision, m_swerveDrive));
    m_eventMap.put("StopIntake", new AutoRunIntakeCube(m_intake, 0, m_vision, m_swerveDrive));
    m_eventMap.put(
        "SetWristIntaking",
        new AutoSetWristDesiredSetpoint(m_wrist, WRIST.SETPOINT.INTAKING_LOW.get()).withTimeout(1));
    m_eventMap.put(
        "SetElevatorIntaking",
        new AutoSetElevatorDesiredSetpoint(m_elevator, ELEVATOR.SETPOINT.INTAKING_LOW.get())
            .withTimeout(1));
    m_eventMap.put(
        "SetWristStowed",
        new AutoSetWristDesiredSetpoint(m_wrist, WRIST.SETPOINT.STOWED.get()).withTimeout(1));
    m_eventMap.put(
        "SetElevatorStowed",
        new AutoSetElevatorDesiredSetpoint(m_elevator, ELEVATOR.SETPOINT.STOWED.get())
            .withTimeout(1));
    m_eventMap.put(
        "SetWristLowConeNode",
        new AutoSetWristDesiredSetpoint(m_wrist, WRIST.SETPOINT.SCORE_LOW_CONE.get())
            .withTimeout(1));
    m_eventMap.put(
        "SetElevatorLowConeNode",
        new AutoSetElevatorDesiredSetpoint(m_elevator, ELEVATOR.SETPOINT.SCORE_LOW_CONE.get())
            .withTimeout(1));
    m_eventMap.put(
        "SetWristMidConeNode",
        new AutoSetWristDesiredSetpoint(m_wrist, WRIST.SETPOINT.SCORE_LOW_CONE.get())
            .withTimeout(1));
    m_eventMap.put(
        "SetElevatorMidConeNode",
        new AutoSetElevatorDesiredSetpoint(m_elevator, ELEVATOR.SETPOINT.SCORE_MID_CONE.get())
            .withTimeout(1));
    m_eventMap.put(
        "SetWristHighConeNode",
        new AutoSetWristDesiredSetpoint(m_wrist, WRIST.SETPOINT.SCORE_LOW_CONE.get())
            .withTimeout(1));
    m_eventMap.put(
        "SetElevatorHighConeNode",
        new AutoSetElevatorDesiredSetpoint(m_elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get())
            .withTimeout(1));
    m_eventMap.put(
        "SetWristLowReverseCubeNode",
        new AutoSetWristDesiredSetpoint(m_wrist, WRIST.SETPOINT.SCORE_LOW_CONE.get())
            .withTimeout(1));
    m_eventMap.put(
        "SetElevatorLowReverseCubeNode",
        new AutoSetElevatorDesiredSetpoint(m_elevator, ELEVATOR.SETPOINT.SCORE_LOW_CONE.get())
            .withTimeout(1));

    m_autoBuilder =
        new SwerveAutoBuilder(
            m_swerveDrive::getPoseMeters,
            m_swerveDrive::setOdometry,
            Constants.SWERVEDRIVE.kSwerveKinematics,
            new PIDConstants(
                Constants.SWERVEDRIVE.kP_Translation,
                Constants.SWERVEDRIVE.kI_Translation,
                Constants.SWERVEDRIVE.kD_Translation),
            new PIDConstants(
                Constants.SWERVEDRIVE.kP_Rotation,
                Constants.SWERVEDRIVE.kI_Rotation,
                Constants.SWERVEDRIVE.kD_Rotation),
            m_swerveDrive::setSwerveModuleStatesAuto,
            m_eventMap,
            true,
            m_swerveDrive);
  }

  public SwerveAutoBuilder getAutoBuilder() {
    return m_autoBuilder;
  }

  /** Use this to pass the autonomous command to the main {@link Robot} class. */
  public void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    m_autoChooser.addOption(
        "BlueTwoTopBalance",
        new TopTwoBalance(
            "BlueTwoTopBalance",
            m_autoBuilder,
            m_swerveDrive,
            m_fieldSim,
            m_wrist,
            m_intake,
            m_vision,
            m_elevator));

    m_autoChooser.addOption(
        "BlueOnePiece",
        new OnePiece(
            "BlueOnePiece",
            m_autoBuilder,
            m_swerveDrive,
            m_fieldSim,
            m_wrist,
            m_intake,
            m_vision,
            m_elevator));

    m_autoChooser.addOption(
        "RedOnePiece",
        new OnePiece(
            "RedOnePiece",
            m_autoBuilder,
            m_swerveDrive,
            m_fieldSim,
            m_wrist,
            m_intake,
            m_vision,
            m_elevator));
    m_autoChooser.addOption(
        "RedTopTwoBalance",
        new TopTwoBalance(
            "RedTwoTopBalance",
            m_autoBuilder,
            m_swerveDrive,
            m_fieldSim,
            m_wrist,
            m_intake,
            m_vision,
            m_elevator));

    //      m_autoChooser.addOption(
    // "AutoLockTest",
    // new AutoLockTest( m_autoBuilder, m_swerveDrive, m_rotationInput, m_rotationInput,
    // m_rotationInput, m_fieldSim, m_wrist));

    // m_autoChooser.addOption(
    //     "RedTopTwoBalance", new RedTopTwoBalance("RedTopTwoCone", m_autoBuilder, m_swerveDrive,
    // m_fieldSim));

    m_autoChooser.addOption(
        "BlueBottomDriveForward",
        new BottomDriveForward(
            "BlueBottomDriveForward",
            m_autoBuilder,
            m_swerveDrive,
            m_fieldSim,
            m_wrist,
            m_intake,
            m_vision,
            m_elevator));

    m_autoChooser.addOption(
        "RedBottomDriveForward",
        new BottomDriveForward(
            "RedBottomDriveForward",
            m_autoBuilder,
            m_swerveDrive,
            m_fieldSim,
            m_wrist,
            m_intake,
            m_vision,
            m_elevator));

    // m_autoChooser.addOption("test", new test(m_autoBuilder, m_swerveDrive, m_fieldSim));

    // m_autoChooser.addOption(
    //     "BlueDriveForward",
    //     new DriveForward("BlueDriveForward", m_autoBuilder, m_swerveDrive, m_fieldSim, m_wrist));

    // m_autoChooser.addOption(
    //     "RedDriveForward",
    //     new DriveForward("RedDriveForward", m_autoBuilder, m_swerveDrive, m_fieldSim, m_wrist));

    // m_autoChooser.setDefaultOption(
    //     "BlueTopDriveForward",
    //     new TopDriveForward(
    //         "BlueTopDriveForward",
    //         m_autoBuilder,
    //         m_swerveDrive,
    //         m_fieldSim,
    //         m_wrist,
    //         m_elevator,
    //         m_intake,
    //         m_vision));

    // m_autoChooser.addOption(
    //     "RedTopDriveForward",
    //     new TopDriveForward(
    //         "RedTopDriveForward",
    //         m_autoBuilder,
    //         m_swerveDrive,
    //         m_fieldSim,
    //         m_wrist,
    //         m_elevator,
    //         m_intake,
    //         m_vision));

    // m_autoChooser.addOption(
    //     "BlueJustBalance", new JustBalance(m_autoBuilder, m_swerveDrive, m_fieldSim, m_wrist));

    SmartDashboard.putData("Auto Selector", m_autoChooser);

    if (RobotBase.isSimulation()) {
      autoPlotter = new SendableChooser<>();
      List<PathPlannerTrajectory> dummy = new ArrayList<>() {};
      dummy.add(new PathPlannerTrajectory());
      autoPlotter.setDefaultOption("None", dummy);
      String[] autos = {
        "BlueTopTwoCone",
        "BlueOnePiece",
        "RedTopTwoCone",
        "BlueBottomDriveForward",
        "RedBottomDriveForward",
        "BlueDriveForward",
        "RedDriveForward"
      };
      for (var auto : autos) {
        var trajectory = TrajectoryUtils.readTrajectory(auto, new PathConstraints(1, 1));
        autoPlotter.addOption(auto, trajectory);
      }

      SmartDashboard.putData("Auto Visualizer", autoPlotter);
    }
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }

  public SwerveDrive getSwerveDrive() {
    return m_swerveDrive;
  }

  public Elevator getElevator() {
    return m_elevator;
  }

  public Wrist getWrist() {
    return m_wrist;
  }

  public Intake getIntake() {
    return m_intake;
  }

  public Vision getVision() {
    return m_vision;
  }

  public Controls getControls() {
    return m_controls;
  }

  public FieldSim getFieldSim() {
    return m_fieldSim;
  }

  public DistanceSensor getDistanceSensor() {
    return m_distanceSensor;
  }

  public void periodic() {
    //    m_distanceSensor.updateSmartDashboard();
    //    m_fieldSim.periodic();
    // Rumbles the controller if the robot is on target based off FieldSim
    xboxController.getHID().setRumble(RumbleType.kBothRumble, m_stateHandler.isOnTarget() ? 1 : 0);
    // m_logManager.periodic();
  }

  public void disabledPeriodic() {
    m_swerveDrive.disabledPeriodic();
  }

  public void testPeriodic() {
    m_stateHandler.testPeriodic();
  }

  public void simulationPeriodic() {
    m_elevator.simulationPeriodic();
    m_memorylog.simulationPeriodic();
    m_fieldSim.setTrajectory(autoPlotter.getSelected());
  }

  @Override
  public void close() throws Exception {
    m_fieldSim.close();
    m_stateHandler.close();
    m_vision.close();
    m_led.close();
    m_wrist.close();
    m_swerveDrive.close();
    m_elevator.close();
    m_intake.close();
    m_controls.close();

    m_distanceSensor.close();
    m_logger.close();
  }
}
