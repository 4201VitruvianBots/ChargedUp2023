package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
import frc.robot.commands.Intake.AutoRunIntakeCube;
import frc.robot.commands.elevator.AutoSetElevatorDesiredSetpoint;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import java.util.List;

// TODO: Rewrite without AutoBuilder
public class TwoPiece extends SequentialCommandGroup {
  private List<PathPlannerTrajectory> m_trajectory;

  public TwoPiece(
      String pathName,
      SwerveAutoBuilder autoBuilder,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator) {

    PathPlannerTrajectory trajectory1 =
        PathPlanner.loadPath("TwoPiece-1", Units.feetToMeters(18), Units.feetToMeters(18), false);

    PathPlannerTrajectory trajectory2 =
        PathPlanner.loadPath("TwoPiece-2", Units.feetToMeters(18), Units.feetToMeters(18), false);

    PPSwerveControllerCommand command1 =
        new PPSwerveControllerCommand(
            trajectory1,
            swerveDrive::getPoseMeters,
            Constants.SWERVEDRIVE.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            true,
            swerveDrive);

    PPSwerveControllerCommand command2 =
        new PPSwerveControllerCommand(
            trajectory2,
            swerveDrive::getPoseMeters,
            Constants.SWERVEDRIVE.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            true,
            swerveDrive);

    addCommands(
        new SetSwerveOdometry(swerveDrive, m_trajectory.get(0).getInitialHolonomicPose(), fieldSim),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())),
        new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive),
        new WaitCommand(0.1),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
            new AutoRunIntakeCone(intake, 0, vision, swerveDrive)),
        new ParallelDeadlineGroup(
            command1,
            new SequentialCommandGroup(
                new WaitCommand(1),
                new ParallelCommandGroup(
                    new AutoSetElevatorDesiredSetpoint(
                        elevator, ELEVATOR.SETPOINT.INTAKING_LOW.get()),
                    new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.INTAKING_LOW.get()),
                    new AutoRunIntakeCube(intake, 0.5, vision, swerveDrive)))),
        new ParallelDeadlineGroup(
            command2,
            new ParallelCommandGroup(
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.INTAKING_LOW.get()),
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.INTAKING_LOW.get()),
                new AutoRunIntakeCube(intake, 0, vision, swerveDrive))),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())),
        new AutoRunIntakeCube(intake, -0.8, vision, swerveDrive),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
            new AutoRunIntakeCone(intake, 0, vision, swerveDrive)),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }

  public List<PathPlannerTrajectory> getTrajectory() {
    return m_trajectory;
  }
}
