// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.USB;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetSwerveDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final SwerveDrive m_swerveDrive = new SwerveDrive();

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
  
    public RobotContainer() {
      initializeSubsystems();
      // initializeAutoChooser();
  
      // Configure the button bindings
      configureButtonBindings();
    }
  
    public void initializeSubsystems() {
      // m_swerveDrive.setDefaultCommand(
      //     new SetSwerveDrive(
      //         m_swerveDrive,
      //         () -> -testController.getLeftY(),
      //         () -> -testController.getLeftX(),
      //         () -> -testController.getRightX()));
  
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
    private void configureButtonBindings() { //TODO: Replace Joystick Button?
      for (int i = 0; i < leftTriggers.length; i++)
        leftTriggers[i] = new JoystickButton(leftJoystick, (i + 1));
      for (int i = 0; i < rightTriggers.length; i++)
        rightTriggers[i] = new JoystickButton(rightJoystick, (i + 1));
      for (int i = 0; i < xBoxTriggers.length; i++)
        xBoxTriggers[i] = new JoystickButton(xBoxController, (i + 1));
      for (int i = 0; i < xBoxPOVTriggers.length; i++)
        xBoxPOVTriggers[i] = new POVButton(xBoxController, (i * 90));
  
    }

  
    

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
