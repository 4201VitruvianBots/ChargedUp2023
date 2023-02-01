package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// creates LED subsystem
public class LED extends SubsystemBase {
  private PieceType pieceIntent = PieceType.NONE;
  private final CANdle m_candle =
      new CANdle(Constants.LED.CANdleID); // LED In constants implecation later (the errors fine)
  int red = 0;
  int green = 0; // setting all LED colors to none: there is no color when robot actiates
  int blue = 0;
  private robotState currentRobotState = robotState.DISABLED;
  private Animation m_Animation = null;

  private final Controls m_controls; // fiugure out during robtoics class

  private final int LEDcount = 22; // TODO: Change LEDCount

  // Create LED strip
  public LED(Controls controls) {
    // sets up LED strip
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true; // sets lights of when the LEDs are activated
    configAll.disableWhenLOS = false; // disables LEDs when robot is off(?)
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar =
        1; // 1 is highest we can go we dont want to blind everyone at the event
    configAll.vBatOutputMode = VBatOutputMode.Modulated; // Modulate
    m_candle.configAllSettings(configAll, 100);

    m_controls = controls;
  }

  /**
   * //will set LED color and animation type
   *
   * @param red
   * @param green
   * @param blue
   * @param white
   * @param speed
   * @param toChange
   */

  // will create LED patterns
  public void setPattern(
      int red, int green, int blue, int white, double speed, robotState toChange) {
        switch (toChange) {
          case INTAKING:
            m_Animation =
              new ColorFlowAnimation(0, 0, 0, 0, 0, LEDcount, Direction.Forward);
            break;
          case ELEVATING: 
            m_Animation =
              new ColorFlowAnimation(0, 0, 0, 0, 0, LEDcount, Direction.Forward);
            break;
          case WRIST: //Solid blue 
            m_Animation =
              new ColorFlowAnimation(66, 95, 255, 0, 0, LEDcount, Direction.Forward);
            break;
          case CONE: //Solid Yellow
            m_Animation =
              new ColorFlowAnimation(250, 246, 17, 0, 0, LEDcount, Direction.Forward);
            break;
          case CUBE: //Solid purple 
            m_Animation =
              new ColorFlowAnimation(255, 0, 255, 0, 0, LEDcount, Direction.Forward);
            break;
          case ENABLED: //Soild green
            m_Animation =
              new ColorFlowAnimation(0, 255, 0, 0, 0, LEDcount, Direction.Forward);
            break;
          case DISABLED: //Solid red
            m_Animation =
              new ColorFlowAnimation(255, 0, 0, 0, 0, LEDcount, Direction.Forward);
            break;
          default:
            break;
        }
      }
 

  @Override
  public void periodic() {
    // null indicates that the animation is "Solid"
    if (m_Animation == null) {
      m_candle.setLEDs(255, 30, 0, 0, 0, 125);
      m_candle.setLEDs(red, green, blue, 0, 20, 35); // setting all LEDs to color
    } else {
      m_candle.animate(m_Animation); // setting the candle animation to m_animation if not null
    }
  }
  ShuffleboardTab ledTab = Shuffleboard.getTab("LED");
    
  public GenericEntry currentState = 
    ledTab.add("LED Mode", currentRobotState.toString()).getEntry();
  

  public void updateShuffleboard() {
    currentState.setString(pieceIntent.name());
  }

  public PieceType getPieceIntent(){
    return pieceIntent;
  }

  public void setPieceIntent(PieceType type){
      pieceIntent = type; 
  }

  /** Different robot states */
  public enum robotState {
    INTAKING,
    ELEVATING,
    WRIST,
    CONE,
    CUBE,
    DISABLED,
    ENABLED, 
  }

  public enum PieceType{
    CONE,
    CUBE,
    NONE,
  }

}
