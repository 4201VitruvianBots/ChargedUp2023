package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
<<<<<<< Updated upstream
=======
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.elevator.AutoSetElevatorDesiredSetpoint;
>>>>>>> Stashed changes
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.simulation.FieldSim;
<<<<<<< Updated upstream
=======
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
>>>>>>> Stashed changes
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;

public class TopTwoCone extends SequentialCommandGroup {
  public TopTwoCone(
<<<<<<< Updated upstream
      String pathName, SwerveAutoBuilder autoBuilder, SwerveDrive swerveDrive, FieldSim fieldSim) {
=======
    String pathName,
    SwerveAutoBuilder autoBuilder,
    SwerveDrive swerveDrive,
    FieldSim fieldSim,
    Wrist wrist,
    Intake intake,
    Elevator elevator) {
>>>>>>> Stashed changes

    var trajectory =
        TrajectoryUtils.readTrajectory(
            pathName, new PathConstraints(Units.feetToMeters(15), Units.feetToMeters(20)));

    var autoPath = autoBuilder.fullAuto(trajectory);

    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),
        new PlotAutoTrajectory(fieldSim, pathName, trajectory),
<<<<<<< Updated upstream
=======
        new ParallelCommandGroup(
          new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.INTAKING_EXTENDED.get()), new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get())),
>>>>>>> Stashed changes
        autoPath,
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
