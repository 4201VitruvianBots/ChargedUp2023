package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;
import java.util.List;

public class DriveForward extends SequentialCommandGroup {
  public DriveForward(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Elevator elevator,
      StateHandler stateHandler) {

    double maxVel = Units.feetToMeters(6);
    double maxAccel = Units.feetToMeters(6);
    if (RobotBase.isSimulation()) {
      maxVel = Units.feetToMeters(4);
      maxAccel = Units.feetToMeters(4);
    }
    PathConstraints constraints = new PathConstraints(maxVel, maxAccel);

    List<PathPlannerTrajectory> trajectories =
        TrajectoryUtils.readTrajectory(pathName, constraints);
    List<PPSwerveControllerCommand> swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, trajectories);

    addCommands(
        new SetSwerveOdometry(swerveDrive, trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new AutoSetSetpoint(stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.STOWED)
            .withTimeout(1),
        new PlotAutoTrajectory(fieldSim, pathName, trajectories),
        swerveCommands.get(0),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
