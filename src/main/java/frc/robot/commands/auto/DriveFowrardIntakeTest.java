package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.AutoRunIntake;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;
import java.util.HashMap;

public class DriveFowrardIntakeTest extends SequentialCommandGroup {
  public DriveFowrardIntakeTest(
      SwerveDrive swerveDrive, FieldSim fieldSim, Intake intake, Wrist wrist) {
    PathPlannerTrajectory trajectory =
        TrajectoryUtils.readTrajectory(
            "DriveFowrardIntakeTest", Units.feetToMeters(2), Units.feetToMeters(2), false);
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

    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("IntakeDown", new AutoRunIntake(intake));
    eventMap.put("PlaceCone", new AutoRunIntake(intake));
    addCommands(
        new SequentialCommandGroup(
            new FollowPathWithEvents(command, trajectory.getMarkers(), eventMap),
            new FollowPathWithEvents(command, trajectory.getMarkers(), eventMap),
            new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
                .andThen(() -> swerveDrive.drive(0, 0, 0, false, false))));
  }
}
