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
import frc.robot.commands.Intake.AutoRunIntakeCube;
import frc.robot.commands.elevator.AutoSetElevatorDesiredSetpoint;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;
import java.util.List;

public class OnePiece extends SequentialCommandGroup {
  private List<PathPlannerTrajectory> m_trajectory;

  public OnePiece(
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
            pathName, new PathConstraints(Units.feetToMeters(9), Units.feetToMeters(9)));

    var autoPath = autoBuilder.fullAuto(m_trajectory);

    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, m_trajectory),
        new ParallelCommandGroup(
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get()),
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
            new AutoRunIntakeCube(intake, -0.5, vision, swerveDrive).withTimeout(0.3)),
        new AutoRunIntakeCube(intake, 0.8, vision, swerveDrive).withTimeout(0.3),
        autoPath,
        new AutoBalance(swerveDrive),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }

  public List<PathPlannerTrajectory> getTrajectory() {
    return m_trajectory;
  }
}
