package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
import frc.robot.commands.Intake.AutoRunIntakeCube;
import frc.robot.commands.elevator.AutoSetElevatorDesiredSetpoint;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import java.util.List;

public class Exhibition extends SequentialCommandGroup {
  private List<PathPlannerTrajectory> m_trajectory;

  public void Exhibiton(
      SwerveDrive swerveDrive, Wrist wrist, Intake intake, Vision vision, Elevator elevator) {


   
    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),
        new RepeatCommand(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get())).withTimeout(4),
            new WaitCommand(1),
            new ParallelCommandGroup(
                    new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.INTAKING_LOW.get()),
                    new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.INTAKING_LOW.get()),
                    new AutoRunIntakeCone(intake, 0.8, vision, swerveDrive))
                .withTimeout(4),
            new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
                new AutoRunIntakeCone(intake, 0, vision, swerveDrive).withTimeout(1)),
            new WaitCommand(1),
            new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_MID_CONE.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_MID_CONE.get())),
            new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive).withTimeout(2),
            new WaitCommand(1),
            new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get())),
            new WaitCommand(1),
    
            // TODO: CUBE
            new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get())),
            new WaitCommand(1),
            new ParallelCommandGroup(
                    new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.INTAKING_LOW.get()),
                    new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.INTAKING_LOW.get()),
                    new AutoRunIntakeCube(intake, 0.65, vision, swerveDrive))
                .withTimeout(4),
            new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
                new AutoRunIntakeCube(intake, 0, vision, swerveDrive).withTimeout(1)),
            new WaitCommand(1),
            new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_MID_CONE.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_MID_CONE.get())),
            new AutoRunIntakeCube(intake, -0.8, vision, swerveDrive).withTimeout(1),
            new WaitCommand(1),
        
            new WaitCommand(1),
            new PrintCommand("hello Tab-O-Auto"))
       
        )
        );
  
}}

