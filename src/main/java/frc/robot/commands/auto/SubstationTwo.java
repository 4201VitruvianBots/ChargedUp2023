package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.INTAKE.INTAKE_SPEEDS;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.commands.intake.AutoSetIntakeSetpoint;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.commands.statehandler.SetSetpoint;
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

public class SubstationTwo extends SequentialCommandGroup {

  public SubstationTwo(
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
            new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH).withTimeout(2),
            new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.HOLDING_CONE).withTimeout(2)),
        new WaitCommand(0.6),
        /** Outakes cone */
        new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.SCORING_CONE).withTimeout(1),
        /** Stows Wrist, Elevator, and Stops intake */
        new ParallelCommandGroup(
            new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED).withTimeout(1.4),
            new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.STOP).withTimeout(1.65)),

        /** Runs Path with Intaking cube during */
        new ParallelDeadlineGroup(
            swerveCommands.get(0),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ParallelCommandGroup(
                    new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.INTAKING_LOW_CUBE)
                        .withTimeout(0.5),
                    new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.INTAKING_CUBE)
                        .withTimeout(0.5)))),
        new ParallelCommandGroup(
            swerveCommands.get(1),
            new SequentialCommandGroup(
                new SetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED).withTimeout(0.5),
                new WaitCommand(0.5),
                new SetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH).withTimeout(2),
                new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.HOLDING_CUBE).withTimeout(2))),

        /** Brings elevator & wrist to High Pulls up cone */
        new WaitCommand(0.8),
        /** Outakes cone */
        new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.SCORING_CUBE).withTimeout(0.3),
        /** Stows Wrist, Elevator, and Stops intake */
        new ParallelCommandGroup(
            new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED).withTimeout(1.4),
            new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.STOP)),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
