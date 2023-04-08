package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.commands.intake.AutoRunIntakeCone;
import frc.robot.commands.intake.AutoRunIntakeCube;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.robot.utils.TrajectoryUtils;
import java.util.List;

public class TwoPiece extends SequentialCommandGroup {

  private final List<PathPlannerTrajectory> m_trajectories;

  public TwoPiece(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    m_trajectories =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(15), Units.feetToMeters(13)));
    //    m_trajectories =
    //        TrajectoryUtils.readTrajectory(
    //            pathName, new PathConstraints(Units.feetToMeters(4), Units.feetToMeters(4)));

    List<PPSwerveControllerCommand> swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    addCommands(
        /** Setting Up Auto Zeros robot to path flips path if necessary */
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),

        /** Brings elevator & wrist to High Pulls up cone */
        new ParallelDeadlineGroup(
            new AutoSetSetpoint(stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.SCORE_HIGH)
                .withTimeout(3),
            new AutoRunIntakeCone(intake, 0.5, vision, swerveDrive)),

        /** Outtakes cone */
        new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive),
        new WaitCommand(1),
        /** Return to Stowed Position */
        new ParallelCommandGroup(
            new AutoSetSetpoint(stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.STOWED)
                .withTimeout(1)),

        /** Runs 2nd part of Path, stows, and holds cube */
        new ParallelDeadlineGroup(
            swerveCommands.get(0),
            new ParallelCommandGroup(
                new AutoSetSetpoint(
                        stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.INTAKING_LOW_CUBE)
                    .withTimeout(1),
                new AutoRunIntakeCube(intake, 0.2, vision, swerveDrive))),
        new WaitCommand(1),
        /** Brings elevator & wrist to High */
        new ParallelDeadlineGroup(
            swerveCommands.get(1),
            new AutoSetSetpoint(stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.SCORE_HIGH)),
        new WaitCommand(1),

        /** Places Cube */
        new AutoRunIntakeCube(intake, -0.8, vision, swerveDrive),
        new WaitCommand(0.7),

        /** Stows and Stops Intake */
        new AutoSetSetpoint(stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.STOWED)
            .withTimeout(3),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }

  public List<PathPlannerTrajectory> getTrajectories() {
    return m_trajectories;
  }
}
