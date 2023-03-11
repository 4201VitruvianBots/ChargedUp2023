package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.Intake.AutoRunIntakeCone;
import frc.robot.commands.elevator.AutoSetElevatorDesiredSetpoint;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RealDoNothing extends SequentialCommandGroup {
  public RealDoNothing(
      Wrist wrist, Intake intake, Vision vision, Elevator elevator, SwerveDrive swerveDrive) {

    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),

        new ParallelCommandGroup(
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
                new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get()),
                new AutoRunIntakeCone(intake, -0.7, vision, swerveDrive))
            .withTimeout(1),
        new ParallelCommandGroup(
                new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
                new AutoSetElevatorDesiredSetpoint(
                    elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()))
            .withTimeout(2.5),
        new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
        new AutoRunIntakeCone(intake, 0.5, vision, swerveDrive).withTimeout(2),
        new ParallelCommandGroup(
            new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()),
            new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.STOWED.get())));
  }
}
