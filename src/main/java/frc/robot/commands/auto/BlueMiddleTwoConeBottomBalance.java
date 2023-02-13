package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.constants.Constants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

public class BlueMiddleTwoConeBottomBalance extends SequentialCommandGroup {
  public BlueMiddleTwoConeBottomBalance(SwerveDrive swerveDrive, FieldSim fieldSim) {
    PathPlannerTrajectory trajectory =
        TrajectoryUtils.readTrajectory(
            "BlueMiddleTwoConeBottomBalance", Units.feetToMeters(2), Units.feetToMeters(2), false);
    PPSwerveControllerCommand command =
        new PPSwerveControllerCommand(
            trajectory,
            swerveDrive::getPoseMeters,
            Constants.constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            false,
            swerveDrive);

    addCommands(
        new PlotAutoTrajectory(fieldSim, trajectory),
        new SetSwerveOdometry(swerveDrive, trajectory.getInitialHolonomicPose(), fieldSim),
        command,
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
