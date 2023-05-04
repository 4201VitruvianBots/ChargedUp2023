package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AUTO.WAIT;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.commands.intake.AutoSetIntakeSetpoint;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;

public class CenterOneBalance extends SequentialCommandGroup {
  public CenterOneBalance(
      String pathName,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Elevator elevator,
      Vision vision,
      StateHandler stateHandler) {

    double maxVel = Units.feetToMeters(8);
    double maxAccel = Units.feetToMeters(8);
    if (RobotBase.isSimulation()) {
      maxVel = Units.feetToMeters(4);
      maxAccel = Units.feetToMeters(4);
    }
    PathConstraints constraints = new PathConstraints(maxVel, maxAccel);

    var trajectories = TrajectoryUtils.readTrajectory(pathName, constraints);
    var swerveCommands =
        TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, trajectories);

    addCommands(
        /** Setting Up Auto Zeros robot to path flips path if necessary */
        new SetSwerveOdometry(swerveDrive, trajectories.get(0).getInitialHolonomicPose(), fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, trajectories),

        /** Brings elevator & wrist to High Pulls up cone */
        new ParallelCommandGroup(
            new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH_CONE)
                .withTimeout(WAIT.SCORE_HIGH_CONE.get()),
            new AutoSetIntakeSetpoint(intake, INTAKE_STATE.HOLDING_CONE, vision, swerveDrive)
                .withTimeout(WAIT.SCORE_HIGH_CONE.get())),
        /** Outakes cone */
        new WaitCommand(WAIT.WAIT_TO_PLACE_CONE.get()),
        new AutoSetIntakeSetpoint(intake, INTAKE_STATE.SCORING_CONE, vision, swerveDrive)
            .withTimeout(WAIT.SCORING_CONE.get()),
        new WaitCommand(WAIT.SCORING_CONE.get()),
        /** Stows Wrist, Elevator, and Stops intake */
        new ParallelCommandGroup(
            new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED)
                .withTimeout(WAIT.STOW_HIGH_CONE.get()),
            new AutoSetIntakeSetpoint(intake, INTAKE_STATE.NONE, vision, swerveDrive)
                .withTimeout(WAIT.STOW_HIGH_CONE.get())),
        new WaitCommand(WAIT.STOW_HIGH_CONE.get()),
        swerveCommands.get(0),
        new AutoBalance(swerveDrive),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
