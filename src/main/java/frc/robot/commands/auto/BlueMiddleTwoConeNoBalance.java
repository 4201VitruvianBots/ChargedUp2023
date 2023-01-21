package frc.robot.commands.auto;

import java.lang.reflect.Field;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;

public class BlueMiddleTwoConeNoBalance extends SequentialCommandGroup {
    public BlueMiddleTwoConeNoBalance (SwerveDrive swerveDrive, FieldSim fieldSim){
        PathPlannerTrajectory trajectory =
        pathplanner.loadPath("BlueMiddleTwoConeNoBalance" ,Units.feetToMeters(2), Units.feetToMeters(2), false);
        PPSwerveControllerCommand command =
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
              new PlotAutoTrajectory(fieldSim,trajectory),
              new SetSwerveOdometry (swerveDrive, trajectory.getInitialPose(), fieldSim),command,
              
              new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
}
}