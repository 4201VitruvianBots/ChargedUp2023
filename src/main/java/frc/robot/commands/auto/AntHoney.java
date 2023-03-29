package frc.robot.commands.auto;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
import frc.robot.commands.Intake.AutoRunIntakeCube;
import frc.robot.commands.elevator.AutoSetElevatorDesiredSetpoint;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class AntHoney extends SequentialCommandGroup {
  public AntHoney(
      SwerveAutoBuilder autoBuilder,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Elevator elevator,
      Vision vision) {

    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),
        new RepeatCommand(
            new SequentialCommandGroup(
        

        new AutoRunIntakeCone(intake, 0, vision, swerveDrive).withTimeout(1),
        new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.INTAKING_LOW.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.INTAKING_LOW.get()),
                new AutoRunIntakeCone(intake, 0.5, vision, swerveDrive)).withTimeout(2),
            new WaitCommand(2),

        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
            new AutoRunIntakeCone(intake, 0, vision, swerveDrive)).withTimeout(3),
        new WaitCommand(0.5),

        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_MID_CONE.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_MID_CONE.get())).withTimeout(5),
        new AutoRunIntakeCone(intake, -0.3, vision, swerveDrive).withTimeout(2),
        new WaitCommand(1),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
        new AutoRunIntakeCone(intake, 0, vision, swerveDrive)).withTimeout(5),
        new WaitCommand(1),

        // TODO: CUBE

        new AutoRunIntakeCube(intake, 0, vision, swerveDrive).withTimeout(1),
        new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.INTAKING_LOW.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.INTAKING_LOW.get()),
                new AutoRunIntakeCube(intake, 0.3, vision, swerveDrive)).withTimeout(2),
            new WaitCommand(2),

        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
            new AutoRunIntakeCube(intake, 0, vision, swerveDrive)).withTimeout(3),
        new WaitCommand(0.5),

        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_MID_CONE.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_MID_CONE.get())).withTimeout(5),
        new AutoRunIntakeCube(intake, -0.3, vision, swerveDrive).withTimeout(2),
        new WaitCommand(1),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
        new AutoRunIntakeCube(intake, 0, vision, swerveDrive)).withTimeout(5),
        new WaitCommand(1)
        )));
      
  }
}
