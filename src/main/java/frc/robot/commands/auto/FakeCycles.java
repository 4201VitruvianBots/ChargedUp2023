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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

public class FakeCycles extends SequentialCommandGroup {
  public FakeCycles(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    double maxVel = Units.feetToMeters(16);
    double maxAccel = Units.feetToMeters(16);
    if (RobotBase.isSimulation()) {
      maxVel = Units.feetToMeters(3);
      maxAccel = Units.feetToMeters(3);
    }
    PathConstraints constraints = new PathConstraints(maxVel, maxAccel);

    var m_trajectories = TrajectoryUtils.readTrajectory(pathName, constraints);
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    addCommands(
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),

        new RepeatCommand(
            new SequentialCommandGroup(
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
     
        /** Runs Path with Intaking cube during */
                new ParallelCommandGroup(
                swerveCommands.get(0),

                new SequentialCommandGroup(
                    new WaitCommand(m_trajectories.get(0).getTotalTimeSeconds() -1.5),
                new ParallelCommandGroup(
                    new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.INTAKING_EXTENDED_CONE),
                    new AutoSetIntakeSetpoint(intake, INTAKE_STATE.INTAKING_CONE, vision, swerveDrive))
                .withTimeout(WAIT.SCORE_HIGH_CONE.get())))
                ,

            /** Stows Wrist, Elevator, and Stops intake */
            new ParallelCommandGroup(
                    new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED),
                    new AutoSetIntakeSetpoint(intake, INTAKE_STATE.NONE, vision, swerveDrive))
                .withTimeout(WAIT.STOW_HIGH_CONE.get()),
               swerveCommands.get(1)
    ))
               );
 

               
    
}
}