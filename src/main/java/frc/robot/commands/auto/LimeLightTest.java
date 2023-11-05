package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.Constants.VISION.CAMERA_SERVER;
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

    m_trajectories =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(4), Units.feetToMeters(4)));

    List<PPSwerveControllerCommand> swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    addCommands(
        /** Setting Up Auto Zeros robot to path flips path if necessary */
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),
        new SetSetpoint(stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.STOWED)
            .withTimeout(2),
        new ParallelDeadlineGroup(
            new WaitCommand(m_trajectories.get(0).getTotalTimeSeconds() + 1),
            new DelayedInterruptingCommand(
                swerveCommands.get(0),
                new DriveForwardWithVisionInput(swerveDrive, vision, () -> 1.25),
                1,
                () -> vision.getValidTarget(CAMERA_SERVER.INTAKE)),
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ParallelCommandGroup(
                    new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.INTAKING_LOW_CUBE),
                    new AutoSetIntakeSetpoint(
                        intake, INTAKE_STATE.INTAKING_CUBE, vision, swerveDrive)))));
  }

  public List<PathPlannerTrajectory> getTrajectories() {
    return m_trajectories;
  }
}
