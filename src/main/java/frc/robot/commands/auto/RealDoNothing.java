package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WRIST;
import frc.robot.commands.wrist.AutoSetWristDesiredSetpoint;
import frc.robot.subsystems.Wrist;

public class RealDoNothing extends SequentialCommandGroup {
  public RealDoNothing(Wrist wrist) {

    addCommands(
        //        new SetSwerveOdometry(swerveDrive, trajectory.get(0).getInitialHolonomicPose(),
        // fieldSim),
        new AutoSetWristDesiredSetpoint(wrist, WRIST.SETPOINT.STOWED.get()));
  }
}
