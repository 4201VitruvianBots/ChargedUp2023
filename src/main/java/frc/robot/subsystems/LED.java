package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//creates LED subsystem
public class LED extends SubsystemBase {
    private final CANdle m_candle = new CANdle(Constants.LED.CANdleID); //LED In constants implecation later (the errors fine)
    int red = 0;
    int green = 0; //setting all LED colors to none: there is no color when robot actiates
    int blue = 0;
    private robotState currentRobotState = robotState.Disabled; 
    private Animation m_Animation = null; 

    private final Controls m_controls; //fiugure out during robtoics class 

    private final int LEDcount = 22; //TODO: Change LEDCount


    //Create LED strip
public LED (Controls controls) {
    //sets up LED strip
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true; //sets lights of when the LEDs are activated
    configAll.disableWhenLOS = false; //disables LEDs when robot is off(?)
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1; //1 is highest we can go we dont want to blind everyone at the event
    configAll.vBatOutputMode = VBatOutputMode.Modulated; //Modulate
    m_candle.configAllSettings(configAll, 100);

    m_controls = controls;
    
}
    
    /** //will set LED color and animation type
    * @param red
    * @param green
    * @param blue
    * @param white
    * @param speed
    * @param toChange
    */

    //will create LED patterns
    public void setPattern( 
      int red, int green, int blue, int white, double speed, AnimationTypes toChange)  {
        switch(toChange){
          case ColorFlow:
            m_Animation =
              new ColorFlowAnimation(red, green, blue, white, speed, LEDcount, Direction.Forward);
            break;
          case Fire:
            m_Animation = 
              new FireAnimation(0.5, 0.7, LEDcount, 0.7, 0.5);
              case Larson: // a line bouncing back and forth with its width determined by size
            m_Animation =
                  new LarsonAnimation(red, green, blue, white, speed, LEDcount, BounceMode.Front, 7);
              break;
            case Rainbow: // neon cat type beat
            m_Animation = new RainbowAnimation(1, speed, LEDcount);
              break;
            case RgbFade: // cycling between red, greed, and blue
            m_Animation = new RgbFadeAnimation(1, speed, LEDcount);
              break;
            case SingleFade: // slowly turn all leds from solid color to off
            m_Animation = new SingleFadeAnimation(red, green, blue, white, speed, LEDcount);
              break;
            case Strobe: // switching between solid color and full off at high speed
            m_Animation = new StrobeAnimation(red, green, blue, white, speed, LEDcount);
              break;
            case Twinkle: // random leds turning on and off with certain color
            m_Animation =
                  new TwinkleAnimation(red, green, blue, white, speed, LEDcount, TwinklePercent.Percent6);
              break;
            case TwinkleOff: // twinkle in reverse
            m_Animation =
                  new TwinkleOffAnimation(
                      red, green, blue, white, speed, LEDcount, TwinkleOffPercent.Percent100);
              break;
            case Solid:
              this.red = red;
              this.green = green;
              this.blue = blue;
              m_Animation = null;
          default:
            System.out.println("Incorrect animation type provided to changeAnimation() method");
        }
    }
    /** 
    *
    *@param state the dominant robot state that LEDs will epxress
    */
     //will set LEDs a coordinated color for an action !TBD!
  public void expressState(robotState state) {
    if(state != currentRobotState){
      switch(state) {
        case Disabled:
            setPattern(225, 0, 0, 0, 0, AnimationTypes.Solid);
            break;
        case Intaking:
          setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Cone:
          setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Cube:
          setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Elevating:
          setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Scoring:
          setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Enabled:
          setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
            break;
		        default:
			      break;
            }
      }
      currentRobotState = state;
       
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
    Shuffleboard.getTab("Controls")
        .add("LED Mode", currentRobotState.toString());
    // ShuffleboardTab.getTab("Controls")
    //   .add("LED Mode", currentRobotState.toString()); 
    // Shuffleboard.addString("Controls", "LED Mode", currentRobotState.toString());
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
    Disabled,
    Intaking,
    Elevating,
    Scoring,
    Cone,
    Cube,
    Enabled, Enabled,
  }
 
}
