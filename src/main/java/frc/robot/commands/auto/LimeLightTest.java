package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;
import java.util.List;

public class LimeLightTest extends SequentialCommandGroup {
  private final List<PathPlannerTrajectory> m_trajectories;

  public LimeLightTest(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    double maxVel = Units.feetToMeters(9);
    double maxAccel = Units.feetToMeters(9);
    double maxVisionVelocity = Units.feetToMeters(3.2);

    m_trajectories =
        TrajectoryUtils.readTrajectory(pathName, new PathConstraints(maxVel, maxAccel));

    List<PPSwerveControllerCommand> swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    addCommands(
        /** Setting Up Auto Zeros robot to path flips path if necessary */
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),
        // new SetSetpoint(stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.INTAKING_LOW_CUBE)
        //     .withTimeout(2),
        // new SetIntakeState(intake, INTAKE_STATE.INTAKING_CUBE),
        // new InstantCommand(() -> vision.setPipeline(CAMERA_SERVER.INTAKE, PIPELINE.CUBE.get())),
        // new InterruptingCommand(
        //     swerveCommands.get(0),
        //     new DriveForwardWithVisionInput(swerveDrive, vision, () -> 0.7).withTimeout(4), //0.4
        // //5
        //     () -> vision.getValidTarget(CAMERA_SERVER.INTAKE)),
        // new SetIntakeState(intake, INTAKE_STATE.HOLDING_CUBE),
        // new SetIntakeState(intake, INTAKE_STATE.NONE),
        // new SetSetpoint(stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.STOWED));

        new InstantCommand(() -> vision.setPipeline(CAMERA_SERVER.INTAKE, PIPELINE.CUBE.get())),

        /** Runs Path with Intaking cube during */
        new ParallelDeadlineGroup(
            new WaitCommand(m_trajectories.get(0).getTotalTimeSeconds() + 0.95),
            new DelayedInterruptingCommand(
                swerveCommands.get(0),
                new DriveForwardWithVisionInput(swerveDrive, vision, () -> maxVisionVelocity),
                1.25,
                () -> vision.getValidTarget(CAMERA_SERVER.INTAKE)),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ParallelCommandGroup(
                    new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.INTAKING_LOW_CUBE),
                    new AutoSetIntakeSetpoint(
                        intake, INTAKE_STATE.INTAKING_CUBE, vision, swerveDrive)))),
        new ParallelCommandGroup(
            swerveCommands.get(1),
            new SetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED)
                .withTimeout(WAIT.INTAKE_TO_STOW.get())));
  }

  public List<PathPlannerTrajectory> getTrajectories() {
    return m_trajectories;
  }
}
