// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.ButtonUI;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.elevator.IncrementElevatorHeight;
import frc.robot.commands.led.AllPurple;
import frc.robot.commands.led.AllYellow;
import frc.robot.commands.led.GetSubsystemStates;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Elevator.elevatorHeights;
import frc.robot.subsystems.LED.robotState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Controls m_controls = new Controls();
  private final Elevator m_elevator = new Elevator();
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);
  private final Intake m_intake = new Intake();
  private final LED m_led = new LED(m_controls);
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  static XboxController xBoxController = new XboxController(Constants.USB.xBoxController);


  public Trigger[] leftTriggers = new Trigger[2];
  public Trigger[] rightTriggers = new Trigger[2];
  public Trigger[] xBoxTriggers = new Trigger[10];
  public Trigger[] xBoxPOVTriggers = new Trigger[4];
  public Trigger xBoxLeftTrigger, xBoxRightTrigger;

    public void initializeSubsystems() {
      m_swerveDrive.setDefaultCommand(
          new SetSwerveDrive( 
              m_swerveDrive,
              () -> -leftJoystick.getRawAxis(1),
              () -> -leftJoystick.getRawAxis(0),
              () -> rightJoystick.getRawAxis(0)));
          m_led.setDefaultCommand(
            new GetSubsystemStates(m_led, m_intake, m_elevator));
              
      
      // Control elevator height by moving the joystick up and down
      m_elevator.setDefaultCommand(
          new IncrementElevatorHeight(
            m_elevator,
            elevatorHeights.JOYSTICK,
            leftJoystick.getRawAxis(1)
          ));
      m_fieldSim.initSim();
    }


  public RobotContainer() {
    initializeSubsystems();
    initializeAutoChooser();

    // Configure the button bindings
    configureBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() { //TODO: Replace Joystick Button?
    for (int i = 0; i < leftTriggers.length; i++)
      leftTriggers[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightTriggers.length; i++)
      rightTriggers[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxTriggers.length; i++)
      xBoxTriggers[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVTriggers.length; i++)
      xBoxPOVTriggers[i] = new POVButton(xBoxController, (i * 90));

    xBoxLeftTrigger =
      new Trigger(() -> xBoxController.getLeftTriggerAxis() > 0.2);
    xBoxLeftTrigger =
      new Trigger(() -> xBoxController.getLeftTriggerAxis() > 0.2);


    xBoxLeftTrigger.whileTrue(new RunIntake(m_intake));
    xBoxLeftTrigger.whileTrue(new AllPurple(m_led, m_intake, robotState.Cube));
    xBoxRightTrigger.whileTrue(new RunIntake(m_intake));
    xBoxRightTrigger.whileTrue(new AllYellow(m_led, m_intake, robotState.Cone));
      
    m_driverController.a().whileTrue(new IncrementElevatorHeight(m_elevator, elevatorHeights.LOW, 0.0));
    m_driverController.b().whileTrue(new IncrementElevatorHeight(m_elevator, elevatorHeights.MID, 0.0));
    m_driverController.y().whileTrue(new IncrementElevatorHeight(m_elevator, elevatorHeights.HIGH, 0.0));

  }


public void disableInit(){
  m_swerveDrive.setNeutralMode(NeutralMode.Coast);
 
}
public void teleopeInit(){
  m_swerveDrive.setNeutralMode(NeutralMode.Brake);
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 public void initializeAutoChooser(){
   m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
//   m_autoChooser.addOption("RedMiddleOneConeBalance", new RedMiddleOneConeBalance(m_swerveDrive, m_fieldSim));
 
  SmartDashboard.putData("Auto Selector", m_autoChooser);
 }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
    
  }
  
  public void periodic() {
    m_fieldSim.periodic();
  }
}
