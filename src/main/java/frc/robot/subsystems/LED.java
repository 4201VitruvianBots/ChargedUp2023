// package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.RobotState;

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
    }
    /** 
    *
    *@param state the dominant robot state that LEDs will epxress
    */
     //will set LEDs a coordinated color for an action !TBD!
  public void expressState(robotState state) {
    if(state != currentRobotState){
      switch(state) {
        case Elavating:
            setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
            break;
        case Scoring:
            setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
            break;
        case Intaking:
                setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
                break;
        case Cone:
            setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
                break;
        case Cube:
            setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
		    case Disabled:
            setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
            break;
		        default:
			      break;
            }
      }
      currentRobotState = state;
       
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
    Scoring,
    Elavating,
    Intaking,
    Disabled,
    Cone,
    Cube,
  }
 
}
