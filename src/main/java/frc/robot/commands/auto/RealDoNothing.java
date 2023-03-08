package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.WRIST;
import frc.robot.commands.elevator.AutoSetElevatorDesiredSetpoint;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class RealDoNothing extends SequentialCommandGroup {
  public RealDoNothing(
    Wrist wrist,
    Elevator elevator) {

    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),
        new AutoSetWristDesiredSetpoint(wrist, Units.degreesToRadians(45.0)),
        new ParallelCommandGroup(
          new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.SCORE_HIGH_CONE.get()),
          new AutoSetElevatorDesiredSetpoint(elevator, ELEVATOR.SETPOINT.SCORE_HIGH_CONE.get()))
   
        
        );
  }
}
