// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.statehandler;

// import edu.wpi.first.networktables.DoubleSubscriber;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.StateHandler;

// public class RunStatehandlerTestMode extends CommandBase {
//   private final StateHandler m_stateHandler;
//   private final DoubleSubscriber
//   STOWED,
//       INTAKING_LOW_CUBE,
//       INTAKING_LOW_CONE,
//       SCORE_LOW_REVERSE,
//       SCORE_LOW_CONE,
//       SCORE_LOW_CUBE,
//       SCORE_MID_CONE,
//       SCORE_MID_CUBE,
//       SCORE_HIGH_CONE,
//       SCORE_HIGH_CUBE,
//       INTAKING_EXTENDED_CONE,
//       INTAKING_EXTENDED_CUBE;

//   private double testSTOWED,
//       testINTAKING_LOW_CUBE,
//       testINTAKING_LOW_CONE,
//       testSCORE_LOW_REVERSE,
//       testSCORE_LOW_CONE,
//       testSCORE_LOW_CUBE,
//       testSCORE_MID_CONE,
//       testSCORE_MID_CUBE,
//       testSCORE_HIGH_CONE,
//       testSCORE_HIGH_CUBE,
//       testINTAKING_EXTENDED_CONE,
//       testINTAKING_EXTENDED_CUBE;
      

//   /** Creates a new RunStatehandlerTestMode. */
//   public RunStatehandlerTestMode(StateHandler stateHandler) {
//    m_stateHandler = stateHandler;
//     addRequirements(m_stateHandler);

//     NetworkTable elevatorNtTab =
//     NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("StateHandlerTestmode");

// // initialize Test Values




//     // initialize Test Values
//     try {
//       elevatorNtTab.getDoubleTopic("STOWED").publish().set(0);

//       elevatorNtTab.getDoubleTopic("INTAKING_LOW_CUBE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("INTAKING_LOW_CONE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("SCORE_LOW_REVERSE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("SCORE_LOW_CONE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("SCORE_LOW_CUBE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("SCORE_MID_CONE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("SCORE_MID_CUBE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("SCORE_HIGH_CONE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("SCORE_HIGH_CUBE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("INTAKING_EXTENDED_CONE").publish().set(0);
//       elevatorNtTab.getDoubleTopic("INTAKING_EXTENDED_CUBE").publish().set(0);
//     } catch (Exception e) {
//       DriverStation.reportError("Error setting default values for elevator test mode", true);
//     }

//     // initialize Test Values
//     testSTOWED = 0;

//     testINTAKING_LOW_CUBE = 0;
//     testINTAKING_LOW_CONE = 0;
//     testSCORE_LOW_REVERSE = 0;
//     testSCORE_LOW_CONE = 0;
//     testSCORE_LOW_CUBE = 0;
//     testSCORE_MID_CONE = 0;
//     testSCORE_MID_CUBE = 0;
//     testSCORE_HIGH_CONE = 0;
//     testSCORE_HIGH_CUBE = 0;
//     testINTAKING_EXTENDED_CONE = 0;
//     testINTAKING_EXTENDED_CUBE = 0;

//     // initialize Subscribers
//     STOWED = elevatorNtTab.getEntry("STOWED").getDouble(0);
//     INTAKING_LOW_CUBE = elevatorNtTab.getEntry("INTAKING_LOW_CUBE").getDouble(0);
//     INTAKING_LOW_CONE = elevatorNtTab.getEntry("INTAKING_LOW_CONE").getDouble(0);
//     SCORE_LOW_REVERSE = elevatorNtTab.getEntry("SCORE_LOW_REVERSE").getDouble(0);
    

//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
