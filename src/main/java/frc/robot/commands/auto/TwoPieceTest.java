package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
import frc.robot.commands.Intake.AutoRunIntakeCube;
import frc.robot.commands.elevator.AutoSetElevatorSetpoint;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.wrist.AutoSetWristSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;
import java.util.List;

public class TwoPieceTest extends SequentialCommandGroup {
  private List<PathPlannerTrajectory> m_trajectory;

  public TwoPieceTest(
      String pathName,
      SwerveAutoBuilder autoBuilder,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator) {

    m_trajectory =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(13), Units.feetToMeters(10)));

    var autoPath = autoBuilder.fullAuto(m_trajectory);

    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),

        new AutoRunIntakeCone(intake, 0, vision, swerveDrive),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectory),
        new ParallelCommandGroup(
            new AutoSetElevatorSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
            new AutoSetWristSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())),
        new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive).withTimeout(1),
        new ParallelCommandGroup(
            new AutoSetElevatorSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_MID_CONE.get()),
            new AutoSetWristSetpoint(wrist, WRIST.SETPOINT.SCORE_MID_CONE.get())),
        autoPath,
        new AutoRunIntakeCone(intake, 0, vision, swerveDrive),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectory),
        new ParallelCommandGroup(
            new AutoSetElevatorSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
            new AutoSetWristSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())),
        new AutoRunIntakeCube(intake, -0.8, vision, swerveDrive).withTimeout(1),
        new ParallelCommandGroup(
            new AutoSetElevatorSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
            new AutoSetWristSetpoint(wrist, WRIST.SETPOINT.STOWED.get())),
        // new AutoRunIntakeCone(intake, 0.2, vision, swerveDrive)),

        // new AutoRunIntakeCone(intake, 0.2, vision, swerveDrive).withTimeout(0.5),
        // new AutoRunIntakeCone(intake, -0.8, vision, swerveDrive).withTimeout(0.5),

        // new ParallelDeadlineGroup(
        //     new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())
        //         .withTimeout(1),
        //     new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get())
        //         .withTimeout(1),
        //     new AutoRunIntakeCone(intake, -0.7, vision, swerveDrive).withTimeout(1)),

        // new ParallelDeadlineGroup(
        //     new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()).withTimeout(1),
        //     new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get())
        //         .withTimeout(1)),
        // new AutoRunIntakeCone(intake, 0, vision, swerveDrive),
        // new PlotAutoTrajectory(fieldSim, pathName, m_trajectory),
        // new ParallelCommandGroup(
        //     new AutoSetElevatorDesiredSetpoint(elevator,
        // ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
        //     new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get())),
        // new WaitCommand(0.1),
        // new AutoRunIntakeCube(intake, -0.8, vision, swerveDrive).withTimeout(1),
        // new WaitCommand(0.5),
        new AutoRunIntakeCone(intake, 0, vision, swerveDrive),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }

  public List<PathPlannerTrajectory> getTrajectory() {
    return m_trajectory;
  }
}