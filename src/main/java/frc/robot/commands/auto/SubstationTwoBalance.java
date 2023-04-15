package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AUTOTIMES.WAIT;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.commands.intake.AutoSetIntakeSetpoint;
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

public class SubstationTwoBalance extends SequentialCommandGroup {

  public SubstationTwoBalance(
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
            new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH_CONE)
                .withTimeout(WAIT.SCORE_HIGH_CONE.get()),
            new AutoSetIntakeSetpoint(intake, INTAKE_STATE.HOLDING_CONE, vision, swerveDrive)
                .withTimeout(WAIT.SCORE_HIGH_CONE.get())),
        /** Outakes cone */
        new AutoSetIntakeSetpoint(intake, INTAKE_STATE.SCORING_CONE, vision, swerveDrive)
            .withTimeout(WAIT.SCORING_CONE.get()),
        /** Stows Wrist, Elevator, and Stops intake */
        new ParallelCommandGroup(
            new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED).withTimeout(0.3),
            new AutoSetIntakeSetpoint(intake, INTAKE_STATE.NONE, vision, swerveDrive)
                .withTimeout(0.3)),
        /** Runs Path with Intaking cube during */
        new ParallelDeadlineGroup(
            swerveCommands.get(0),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ParallelCommandGroup(
                    new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.INTAKING_LOW_CUBE)
                        .withTimeout(0.5),
                    new AutoSetIntakeSetpoint(
                            intake, INTAKE_STATE.INTAKING_CUBE, vision, swerveDrive)
                        .withTimeout(0.5)))),
        new ParallelCommandGroup(
            swerveCommands.get(1),
            new SequentialCommandGroup(
                new SetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED).withTimeout(0.5),
                new WaitCommand(0.5),
                new SetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH_CONE)
                    .withTimeout(1.4),
                new AutoSetIntakeSetpoint(intake, INTAKE_STATE.HOLDING_CUBE, vision, swerveDrive)
                    .withTimeout(1.4))),

        /** Brings elevator & wrist to High Pulls up cone */
        new WaitCommand(0.4),
        /** Outakes cone */
        new AutoSetIntakeSetpoint(intake, INTAKE_STATE.SCORING_CUBE, vision, swerveDrive)
            .withTimeout(0.3),
        /** Stows Wrist, Elevator, and Stops intake */
        new ParallelCommandGroup(
            new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED).withTimeout(0.5),
            new AutoSetIntakeSetpoint(intake, INTAKE_STATE.NONE, vision, swerveDrive)
                .withTimeout(0.5)),
        swerveCommands.get(2),
        new AutoBalance(swerveDrive),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
