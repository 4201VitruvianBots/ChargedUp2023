package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AUTO.WAIT;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.Constants.VISION.PIPELINE;
import frc.robot.commands.DelayedInterruptingCommand;
import frc.robot.commands.intake.AutoSetIntakeSetpoint;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.commands.statehandler.SetSetpoint;
import frc.robot.commands.swerve.DriveForwardWithVisionInput;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.TrajectoryUtils;

public class BumpOnePickUp extends SequentialCommandGroup {

  public BumpOnePickUp(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    double maxVel = Units.feetToMeters(6);
    double maxAccel = Units.feetToMeters(6);
    if (RobotBase.isSimulation()) {
      maxVel = Units.feetToMeters(4);
      maxAccel = Units.feetToMeters(4);
    }
    PathConstraints constraints = new PathConstraints(maxVel, maxAccel);

    var m_trajectories = TrajectoryUtils.readTrajectory(pathName, constraints);
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    addCommands(
        /** Setting Up Auto Zeros robot to path flips path if necessary */
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),

        /** Brings elevator & wrist to High Pulls up cone */
        new ParallelCommandGroup(
                new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH_CONE),
                new AutoSetIntakeSetpoint(intake, INTAKE_STATE.HOLDING_CONE, vision, swerveDrive))
            .withTimeout(WAIT.SCORE_HIGH_CONE.get()),
        /** Outakes cone */
        new WaitCommand(WAIT.WAIT_TO_PLACE_CONE.get()),
        new AutoSetIntakeSetpoint(intake, INTAKE_STATE.SCORING_CONE, vision, swerveDrive)
            .withTimeout(WAIT.SCORING_CONE.get()),
        new WaitCommand(WAIT.SCORING_CONE.get()),
        /** Stows Wrist, Elevator, and Stops intake */
        new ParallelCommandGroup(
                new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED),
                new AutoSetIntakeSetpoint(intake, INTAKE_STATE.NONE, vision, swerveDrive))
            .withTimeout(WAIT.STOW_HIGH_CONE.get()),
        new InstantCommand(() -> vision.setPipeline(CAMERA_SERVER.INTAKE, PIPELINE.CUBE.get())),

        /** Runs Path with Intaking cube during */
        new ParallelDeadlineGroup(
            new WaitCommand(m_trajectories.get(0).getTotalTimeSeconds() + 0.5),
            new DelayedInterruptingCommand(
                swerveCommands.get(0),
                new DriveForwardWithVisionInput(swerveDrive, vision, () -> 1),
                1.5,
                () -> vision.getValidTarget(CAMERA_SERVER.INTAKE)),
            new SequentialCommandGroup(
                new WaitCommand(1.25),
                new ParallelCommandGroup(
                    new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.INTAKING_LOW_CUBE),
                    new AutoSetIntakeSetpoint(
                        intake, INTAKE_STATE.INTAKING_CUBE, vision, swerveDrive)))),

        /** Stows and Stops Intake */
        new ParallelCommandGroup(
                new SetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED),
                new AutoSetIntakeSetpoint(intake, INTAKE_STATE.HOLDING_CUBE, vision, swerveDrive))
            .withTimeout(0.5),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
