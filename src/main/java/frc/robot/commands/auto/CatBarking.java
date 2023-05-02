package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.orchestra.SetSong;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TalonOrchestra;

public class CatBarking extends SequentialCommandGroup {
  public CatBarking(TalonOrchestra orchestra, String songName, SwerveDrive swerveDrive) {

    addCommands(
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
        new SetSong(orchestra, songName));
  }
}
