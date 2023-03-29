package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;

public class JustBalance extends SequentialCommandGroup {
  public JustBalance(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Elevator elevator,
      Vision vision) {

    var trajectories =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(6)));
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, trajectories);

    addCommands(
        // TODO: Reset Odometry
        new PlotAutoTrajectory(fieldSim, pathName, trajectories),
        // new AutoRunIntakeCone(intake, 0, vision, swerveDrive),
        //     new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),

        // new ParallelCommandGroup(
        //     new AutoSetElevatorDesiredSetpoint(elevator,
        // ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
        //     new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())),
        // new WaitCommand(0.1),
        // new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive).withTimeout(1),
        // new WaitCommand(0.5),
        // new ParallelCommandGroup(
        //     new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
        //     new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get())),
        // autoPath,
        new AutoBalance(swerveDrive),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
