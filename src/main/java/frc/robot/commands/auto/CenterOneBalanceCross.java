package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.AutoRunIntakeCone;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.commands.statehandler.SetSetpoint;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;

// TODO: Rewrite without AutoBuilder
public class CenterOneBalanceCross extends SequentialCommandGroup {
  public CenterOneBalanceCross(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Elevator elevator,
      Vision vision,
      StateHandler stateHandler) {

    var m_trajectories =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(6)));
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    addCommands(
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),

        /** Brings elevator & wrist to High Pulls up cone */
        new ParallelCommandGroup(
            new AutoSetSetpoint(
                    stateHandler,
                    elevator,
                    wrist,
                    frc.robot.Constants.STATE_HANDLER.SETPOINT.SCORE_HIGH)
                .withTimeout(2),
            new AutoRunIntakeCone(intake, 0.5, vision, swerveDrive).withTimeout(2)),
        new WaitCommand(0.6),
        /** Outakes cone */
        new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive).withTimeout(1),
        /** Stows Wrist, Elevator, and Stops intake */
        new ParallelCommandGroup(
            new AutoSetSetpoint(
                    stateHandler,
                    elevator,
                    wrist,
                    frc.robot.Constants.STATE_HANDLER.SETPOINT.STOWED)
                .withTimeout(1.8),
            new AutoRunIntakeCone(intake, 0, vision, swerveDrive).withTimeout(1.8)),
        // new WaitCommand(0.25),
        /** Runs Path with Intaking cube during */
        new ParallelDeadlineGroup(
            swerveCommands.get(0),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ParallelCommandGroup(
                    new AutoSetSetpoint(
                            stateHandler,
                            elevator,
                            wrist,
                            frc.robot.Constants.STATE_HANDLER.SETPOINT.INTAKING_LOW_CUBE)
                        .withTimeout(0.5),
                    new AutoRunIntakeCone(intake, 0.5, vision, swerveDrive).withTimeout(0.5)))),
        new ParallelCommandGroup(
            swerveCommands.get(1),
            new SequentialCommandGroup(
                new SetSetpoint(
                        stateHandler,
                        elevator,
                        wrist,
                        frc.robot.Constants.STATE_HANDLER.SETPOINT.STOWED)
                    .withTimeout(0.5))),
        new AutoBalance(swerveDrive),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
