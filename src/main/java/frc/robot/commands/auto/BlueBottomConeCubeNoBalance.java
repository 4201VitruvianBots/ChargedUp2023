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
  public BlueBottomConeCubeNoBalance(Intake intake, Wrist wrist, SwerveDrive swerveDrive, FieldSim fieldSim) {
    PathPlannerTrajectory trajectory =
        TrajectoryUtils.readTrajectory(
            "BlueBottomConeCubeNoBalance", Units.feetToMeters(2), Units.feetToMeters(2), false);
    PPSwerveControllerCommand command1 =
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

    addCommands(
        new PlotAutoTrajectory(fieldSim, trajectory),
        new SetSwerveOdometry(swerveDrive, trajectory.getInitialHolonomicPose(), fieldSim),
        command1,
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
        // Outtake Cone
        new AutoRunIntake(intake),
        new AutoRunWrist(wrist),
        command1.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
