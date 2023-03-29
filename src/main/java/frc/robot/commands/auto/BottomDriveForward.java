package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
import frc.robot.commands.elevator.AutoSetElevatorDesiredSetpoint;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;

// TODO: Rewrite without AutoBuilder
public class BottomDriveForward extends SequentialCommandGroup {

  public BottomDriveForward(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator) {

    var trajectories =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(6)));
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, trajectories);

    addCommands(
        // TODO: Reset Odometry
        new AutoRunIntakeCone(intake, 0, vision, swerveDrive),
        new PlotAutoTrajectory(fieldSim, pathName, trajectories),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())),
        new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive).withTimeout(1),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get())),
        //        autoPath,
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())),
        new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive).withTimeout(1),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get())),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
//
