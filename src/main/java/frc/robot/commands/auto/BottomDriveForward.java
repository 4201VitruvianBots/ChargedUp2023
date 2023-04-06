package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
import frc.robot.commands.Intake.AutoRunIntakeCube;
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

// TODO: Rewrite without AutoBuilder
public class BottomDriveForward extends SequentialCommandGroup {

  public BottomDriveForward(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    var trajectories =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(6)));
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, trajectories);

    addCommands(
        /** Setting Up Auto Zeros robot to path flips path if nessesary */
        new SetSwerveOdometry(swerveDrive, trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, trajectories),

        /** Brings elevator & wrist to High Pulls up cone */
        new ParallelCommandGroup( 
            new SetSetpoint(stateHandler, elevator, wrist, frc.robot.Constants.STATE_HANDLER.SETPOINT.SCORE_HIGH),
            new AutoRunIntakeCone(intake, 0.5, vision, swerveDrive)),

        /** Outakes cone */
        new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive),
        new WaitCommand(1),

        /** Stows Wrist, Elevator, and Stops intake */
        new ParallelCommandGroup(
            new SetSetpoint(stateHandler, elevator, wrist, frc.robot.Constants.STATE_HANDLER.SETPOINT.STOWED),
            new AutoRunIntakeCone(intake, 0, vision, swerveDrive)),

        // /** TODO: test this implementation out
        //  * Stows Wrist, Elevator, and Stops intake
        //  *  After 1 second, start Path
        // */
        //     new ParallelDeadlineGroup(
        //         new ParallelCommandGroup(
        //             new AutoSetElevatorDesiredSetpoint(
        //                 elevator, ELEVATOR.SETPOINT.STOWED.get()),
        //             new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
        //             new AutoRunIntakeCube(intake, 0, vision, swerveDrive)),

        //             new SequentialCommandGroup(
        //                 new WaitCommand(1),
        //                 swerveCommands.get(0)
        //                 )),

        /** Runs Path with Intaking cube during */
        new ParallelDeadlineGroup(
            swerveCommands.get(0),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ParallelCommandGroup(
                new SetSetpoint(stateHandler, elevator, wrist, frc.robot.Constants.STATE_HANDLER.SETPOINT.INTAKING_LOW_CUBE),
                    new AutoRunIntakeCube(intake, 0.5, vision, swerveDrive)))),

        /** Runs 2nd part of Path, stows, and holds cube */
        new ParallelDeadlineGroup(
            swerveCommands.get(1),
            new ParallelCommandGroup(

            new SetSetpoint(stateHandler, elevator, wrist, frc.robot.Constants.STATE_HANDLER.SETPOINT.STOWED),
                new AutoRunIntakeCube(intake, 0.2, vision, swerveDrive))),

        /** Brings elevator & wrist to High */

        new SetSetpoint(stateHandler, elevator, wrist, frc.robot.Constants.STATE_HANDLER.SETPOINT.SCORE_HIGH),

        /** Places Cube */
        new AutoRunIntakeCube(intake, -0.8, vision, swerveDrive),
        new WaitCommand(0.7),
        /** Stows and Stops Intake */
        new ParallelCommandGroup(

        new SetSetpoint(stateHandler, elevator, wrist, frc.robot.Constants.STATE_HANDLER.SETPOINT.STOWED),
            new AutoRunIntakeCone(intake, 0, vision, swerveDrive)),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
//
