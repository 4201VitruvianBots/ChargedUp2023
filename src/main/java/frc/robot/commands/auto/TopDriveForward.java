package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
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

public class TopDriveForward extends SequentialCommandGroup {
  public TopDriveForward(
      String pathName,
      SwerveAutoBuilder autoBuilder,
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Elevator elevator,
      Intake intake,
      Vision vision) {

    var trajectory =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(6), Units.feetToMeters(6)));

    var autoPath = autoBuilder.fullAuto(trajectory);
    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, trajectory),
        new ParallelCommandGroup(
                new AutoSetWristSetpoint(wrist, WRIST.SETPOINT.INTAKING_LOW_CUBE.get()),
                // new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get()),
                // new AutoSetElevatorDesiredSetpoint(elevator,
                // ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()),
                new AutoRunIntakeCone(intake, -0.7, vision, swerveDrive))
            .withTimeout(1),
        autoPath,
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
        new ParallelCommandGroup(
                new AutoSetWristSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
                new AutoSetElevatorSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
                new AutoRunIntakeCone(intake, 0.7, vision, swerveDrive))
            .withTimeout(1));
  }
}
