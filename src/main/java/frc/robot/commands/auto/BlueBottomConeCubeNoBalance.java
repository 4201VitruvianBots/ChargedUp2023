package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.AutoRunIntake;
import frc.robot.commands.Intake.AutoRunWrist;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;

public class BlueBottomConeCubeNoBalance extends SequentialCommandGroup {
  public BlueBottomConeCubeNoBalance(
      SwerveDrive swerveDrive, FieldSim fieldSim, Intake intake, Wrist wrist) {
    PathPlannerTrajectory trajectory =
        TrajectoryUtils.readTrajectory(
            "BlueBottomConeCubeNoBalance", Units.feetToMeters(2), Units.feetToMeters(2), false);
    PPSwerveControllerCommand command =
        new PPSwerveControllerCommand(
            trajectory,
            swerveDrive::getPoseMeters,
            Constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            false,
            swerveDrive);

    PPSwerveControllerCommand command2 =
        new PPSwerveControllerCommand(
            trajectory,
            swerveDrive::getPoseMeters,
            Constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            swerveDrive);

    PPSwerveControllerCommand command3 =
        new PPSwerveControllerCommand(
            trajectory,
            swerveDrive::getPoseMeters,
            Constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            swerveDrive);

    addCommands(
        new PlotAutoTrajectory(fieldSim, trajectory),
        new SetSwerveOdometry(swerveDrive, trajectory.getInitialHolonomicPose(), fieldSim),
        command,
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),

        // PATH 1 OutTakeCOne
        new AutoRunIntake(intake),
        new AutoRunWrist(wrist),
        command.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
        // PATH 2 Intake Cube
        new AutoRunIntake(intake),
        new AutoRunWrist(wrist),
        command2.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
        // PATH 3 OutTake Cube
        new AutoRunIntake(intake),
        new AutoRunWrist(wrist),
        command3.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
