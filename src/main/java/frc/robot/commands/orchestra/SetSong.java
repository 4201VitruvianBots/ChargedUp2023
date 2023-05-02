package frc.robot.commands.orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TalonOrchestra;

public class SetSong extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TalonOrchestra m_orchestra;

  private String m_song;

  public SetSong(TalonOrchestra orchestra, String song) {
    m_orchestra = orchestra;
    m_song = song;
    addRequirements(orchestra);
  }

  @Override
  public void initialize() {
  
  }

  @Override
  public void execute() {
    
    m_orchestra.song(m_song);
  }

  @Override
  public void end(boolean interrupted) {
    // stops the song
    m_orchestra.stopSong();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
