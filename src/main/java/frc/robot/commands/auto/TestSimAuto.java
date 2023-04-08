package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.commands.elevator.SetElevatorSetpoint;
import frc.robot.commands.statehandler.SetSetpoint;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.commands.wrist.SetWristSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;
import java.util.List;

public class TestSimAuto extends SequentialCommandGroup {
  List<PathPlannerTrajectory> m_trajectories;

  public TestSimAuto(
      String pathName,
      SwerveDrive swerveDrive,
      Elevator elevator,
      Wrist wrist,
      StateHandler stateHandler,
      FieldSim fieldSim) {
    m_trajectories =
        TrajectoryUtils.readTrajectory(
            pathName,
            new PathConstraints(
                SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.5,
                SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.5));

    List<PPSwerveControllerCommand> swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    var testWait = new WaitCommand(8);
    testWait.addRequirements(swerveDrive);
    addCommands(
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),
        swerveCommands.get(0),
        swerveCommands.get(1),
        swerveCommands.get(2),
        new SetSetpoint(stateHandler, elevator, wrist, Constants.STATE_HANDLER.SETPOINT.SCORE_HIGH).withTimeout(3),
        swerveCommands.get(3),
        new RepeatCommand(
            new InstantCommand(() -> swerveDrive.drive(0, 0, 0, false, false), swerveDrive)));
  }

  public List<PathPlannerTrajectory> getTrajectories() {
    return m_trajectories;
  }
}
