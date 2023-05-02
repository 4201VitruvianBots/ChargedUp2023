package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TalonOrchestra extends SubsystemBase {

  private final TalonFX[] talons = {
    
    new TalonFX(Constants.CAN.wristMotor),
    new TalonFX(Constants.CAN.intakeMotor),
    new TalonFX(Constants.CAN.backLeftDriveMotor),
    new TalonFX(Constants.CAN.backRightDriveMotor),
    new TalonFX(Constants.CAN.frontLeftDriveMotor),
    new TalonFX(Constants.CAN.frontRightDriveMotor)
    
  };

  public Orchestra m_orchestra = new Orchestra();

  public TalonOrchestra() {
    
    for (TalonFX talon : talons) { //for every talon in the Talon Array...
      m_orchestra.addInstrument(talon);
    }
    m_orchestra.addInstrument(
      new TalonFX(Constants.CAN.intakeMotor));
   
  }
  

  public void song(String name) {
    if (!m_orchestra.isPlaying()) {
      m_orchestra.loadMusic(name);
      m_orchestra.play();
    }
  }

  public void stopSong() {
    m_orchestra.stop();
    
  }

 public void periodic() {}
  
  }