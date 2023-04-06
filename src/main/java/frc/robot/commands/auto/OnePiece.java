package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
import frc.robot.commands.Intake.AutoRunIntakeCube;
import frc.robot.commands.elevator.AutoSetElevatorSetpoint;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;
import java.util.List;

// TODO: Rewrite without AutoBuilder
public class OnePiece extends SequentialCommandGroup {
  private List<PathPlannerTrajectory> m_trajectories;

  public OnePiece(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator) {

    m_trajectories =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(6)));
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    addCommands(
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new AutoRunIntakeCone(intake, 0, vision, swerveDrive),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),
        new ParallelCommandGroup(
            new AutoSetElevatorSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
            new AutoSetWristSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())),
        new WaitCommand(0.5),
        new AutoRunIntakeCube(intake, -0.8, vision, swerveDrive).withTimeout(1),
        new WaitCommand(1.5),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get())),
        // autoPath,
        new AutoBalance(swerveDrive),

        // new AutoRunIntakeCone(intake, 0.2, vision, swerveDrive)),

        // new AutoRunIntakeCone(intake, 0.2, vision, swerveDrive).withTimeout(0.5),
        // new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive).withTimeout(0.5),

        // new ParallelDeadlineGroup(
        //     new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())
        //         .withTimeout(1),
        //     new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get())
        //         .withTimeout(1),
        //     new AutoRunIntakeCone(intake, -0.7, vision, swerveDrive).withTimeout(1)),

        // new ParallelDeadlineGroup(
        //     new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()).withTimeout(1),
        //     new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get())
        //         .withTimeout(1)),

        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }

  public List<PathPlannerTrajectory> getTrajectory() {
    return m_trajectories;
  }
}
