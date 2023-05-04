package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.Constants.VISION.PIPELINE;
import frc.robot.commands.DelayedInterruptingCommand;
import frc.robot.commands.intake.AutoSetIntakeSetpoint;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.commands.swerve.DriveForwardWithVisionInput;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

public class ExampleLimelightTrajectory extends SequentialCommandGroup {
  public ExampleLimelightTrajectory(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    double maxVel = Units.feetToMeters(16);
    double maxAccel = Units.feetToMeters(13);
    if (RobotBase.isSimulation()) {
      maxVel = Units.feetToMeters(4);
      maxAccel = Units.feetToMeters(4);
    }
    PathConstraints constraints = new PathConstraints(maxVel, maxAccel);

    var m_trajectories = TrajectoryUtils.readTrajectory(pathName, constraints);
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);

    addCommands(
        new SetSwerveOdometry(
            swerveDrive, m_trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectories),
        new InstantCommand(() -> vision.setPipeline(CAMERA_SERVER.INTAKE, PIPELINE.CUBE.get())),
        new ParallelDeadlineGroup(
            // Use the trajectory's normal time as the deadline
            new WaitCommand(m_trajectories.get(0).getTotalTimeSeconds()),
            // Run normal auto for 3 seconds, then check if we should interrupt with limelight
            new DelayedInterruptingCommand(
                swerveCommands.get(0),
                new DriveForwardWithVisionInput(swerveDrive, vision, () -> 0.4),
                3, // Put the delay for the limelight here
                () -> vision.getValidTarget(CAMERA_SERVER.INTAKE)),
            // Move the wrist/intake at the same time.
            new SequentialCommandGroup(
                new WaitCommand(0.75),
                new ParallelCommandGroup(
                    new AutoSetSetpoint(
                        stateHandler, elevator, wrist, STATE_HANDLER.SETPOINT.INTAKING_LOW_CUBE),
                    new AutoSetIntakeSetpoint(
                        intake, INTAKE_STATE.INTAKING_CUBE, vision, swerveDrive)))));
  }
}
