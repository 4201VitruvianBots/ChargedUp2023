package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
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
  private robotState currentRobotState = robotState.Disabled;
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
      int red, int green, int blue, int white, double speed, AnimationTypes toChange) {}
  /**
   * @param state the dominant robot state that LEDs will epxress
   */
  // will set LEDs a coordinated color for an action !TBD!
  public void expressState(robotState state) {
    if (state != currentRobotState) {
      switch (state) {
        case Intaking:
          setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Elevating: 
          setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Wrist: //Solid blue 
          setPattern(66, 95, 255, 0, 0, AnimationTypes.Solid);
          break;
        case Cone: //Solid Yellow
          setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Cube: //Solid purple 
          setPattern(255, 0, 255, 0, 0, AnimationTypes.Solid);
          break;
          case Enabled: //Soild green
          setPattern(0, 255, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Disabled: //Solid red
          setPattern(255, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        default:
          break;
      }
    }
    currentRobotState = state;
  }

  public PieceType getPieceIntent(){
    return pieceIntent;
  }

  public void setPieceIntent(PieceType type){
      pieceIntent = type; 
  }

  /** Different LED animation types */
  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    Solid
  }

  /** Different robot states */
  public enum robotState {
    Intaking,
    Elevating,
    Wrist,
    Cone,
    Cube,
    Disabled,
    Enabled, 
  }

  public enum PieceType{
    CONE,
    CUBE,
    NONE,
  }
}
