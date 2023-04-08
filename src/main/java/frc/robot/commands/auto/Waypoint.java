package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

public class Waypoint extends SequentialCommandGroup {
  public Waypoint(SwerveAutoBuilder autoBuilder, SwerveDrive swerveDrive, FieldSim fieldSim) {
    var trajectory =
        TrajectoryUtils.readTrajectory(
            "NewPath", new PathConstraints(Units.feetToMeters(2), Units.feetToMeters(2)));

    var autoPath = autoBuilder.fullAuto(trajectory);
    autoPath.addRequirements(swerveDrive);

    addCommands(
        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(), fieldSim),
        new PrintCommand("Test"),
        new PrintCommand("TS: " + trajectory.get(0).getInitialHolonomicPose()),
        new PrintCommand("TE: " + trajectory.get(0).getEndState().poseMeters),
        new PrintCommand("P " + swerveDrive.getPoseMeters()),
        autoPath);
  }
}
