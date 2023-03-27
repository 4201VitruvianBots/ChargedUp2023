package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
import frc.robot.commands.Intake.AutoRunIntakeCube;
import frc.robot.commands.elevator.AutoSetElevatorDesiredSetpoint;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;

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
        
        new AutoRunIntakeCone(intake, 0, vision, swerveDrive).withTimeout(1),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.INTAKING_LOW.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.INTAKING_LOW.get()),
        new AutoRunIntakeCone(intake, 0.8, vision, swerveDrive)).withTimeout(10),

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
             new AutoRunIntakeCone(intake, 0, vision, swerveDrive).withTimeout(1),
        new WaitCommand(1),

//TODO: CUBE


new ParallelCommandGroup(
    new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.INTAKING_LOW.get()),
    new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.INTAKING_LOW.get()),
new AutoRunIntakeCube(intake, 0.65, vision, swerveDrive)).withTimeout(4),

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

new ParallelCommandGroup(
    new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
    new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get())),
new WaitCommand(1)
    );
    
 }
}
