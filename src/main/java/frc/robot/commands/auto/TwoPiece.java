package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.AutoRunIntakeCone;
import frc.robot.commands.intake.AutoRunIntakeCube;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.commands.statehandler.SetSetpoint;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.robot.utils.TrajectoryUtils;

public class TwoPiece extends SequentialCommandGroup {

  public TwoPiece(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    var m_trajectories =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(16), Units.feetToMeters(13)));
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
                .withTimeout(1.4),
            new AutoRunIntakeCone(intake, 0, vision, swerveDrive).withTimeout(1.65)),

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
                    new AutoRunIntakeCube(intake, 0.5, vision, swerveDrive).withTimeout(0.5)))),
        new ParallelCommandGroup(
            swerveCommands.get(1),
            new SequentialCommandGroup(
                new SetSetpoint(
                        stateHandler,
                        elevator,
                        wrist,
                        frc.robot.Constants.STATE_HANDLER.SETPOINT.STOWED)
                    .withTimeout(0.5),
                new WaitCommand(0.5),
                new SetSetpoint(
                        stateHandler,
                        elevator,
                        wrist,
                        frc.robot.Constants.STATE_HANDLER.SETPOINT.SCORE_HIGH)
                    .withTimeout(2),
                new AutoRunIntakeCube(intake, 0.2, vision, swerveDrive).withTimeout(2))),

        /** Brings elevator & wrist to High Pulls up cone */
        new WaitCommand(0.8),
        /** Outakes cone */
        new AutoRunIntakeCube(intake, -0.8, vision, swerveDrive).withTimeout(0.3),
        /** Stows Wrist, Elevator, and Stops intake */
        new ParallelCommandGroup(
            new AutoSetSetpoint(
                    stateHandler,
                    elevator,
                    wrist,
                    frc.robot.Constants.STATE_HANDLER.SETPOINT.STOWED)
                .withTimeout(1.4),
            new AutoRunIntakeCone(intake, 0, vision, swerveDrive)),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
