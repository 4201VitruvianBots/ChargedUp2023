package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LED.*;
import frc.robot.Constants.STATEHANDLER.INTAKING_STATES;

// creates LED subsystem
public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  private INTAKING_STATES pieceIntent = INTAKING_STATES.NONE;
  private final CANdle m_candle = new CANdle(Constants.CAN.CANdle); // LED In constants)
  int red = 0;
  int green = 0; // setting all LED colors to none: there is no color when robot activates
  int blue = 0;
  private LED_STATE currentRobotState = LED_STATE.DISABLED;
  private Animation m_toAnimate = null;

  private final Controls m_controls; // figure out during robotics class

  private final int LEDcount = 72;

  private final StringPublisher ledStatePub;
  // Create LED strip
  public LEDSubsystem(Controls controls) {
    m_candle.configFactoryDefault(); // sets up LED strip
    // sets up LED strip
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true; // sets lights of when the LEDs are activated
    configAll.disableWhenLOS = false; // disables LEDs when robot is off(?)
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar =
        0.75; // 1 is highest we can go we don't want to blind everyone at the event
    configAll.vBatOutputMode = VBatOutputMode.Modulated; // Modulate
    m_candle.configAllSettings(configAll, 100);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_2_Startup, 255);
    m_candle.setStatusFramePeriod(
        CANdleStatusFrame.CANdleStatusFrame_Status_3_FirmwareApiStatus, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_4_ControlTelem, 255);
    m_candle.setStatusFramePeriod(
        CANdleStatusFrame.CANdleStatusFrame_Status_5_PixelPulseTrain, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_6_BottomPixels, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_7_TopPixels, 255);
    m_controls = controls;
    var nt_instance =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Controls");
    ledStatePub = nt_instance.getStringTopic("LED State").publish();
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
      int red, int green, int blue, int white, double speed, ANIMATION_TYPE toChange) {

    switch (toChange) {
      case ColorFlow: // stripe of color flowing through the led strip
        m_toAnimate =
            new ColorFlowAnimation(red, green, blue, white, speed, LEDcount, Direction.Forward);
        break;
      case Fire: // red and orange leds flaming up and down the led strip
        m_toAnimate = new FireAnimation(0.5, 0.7, LEDcount, 0.7, 0.5);
        break;
      case Larson: // a line bouncing back and forth with its width determined by size
        m_toAnimate =
            new LarsonAnimation(red, green, blue, white, speed, LEDcount, BounceMode.Front, 7);
        break;
      case Rainbow: // neon cat type beat
        m_toAnimate = new RainbowAnimation(1, speed, LEDcount);
        break;
      case RgbFade: // cycling between red, greed, and blue
        m_toAnimate = new RgbFadeAnimation(1, speed, LEDcount);
        break;
      case SingleFade: // slowly turn all leds from solid color to off
        m_toAnimate = new SingleFadeAnimation(red, green, blue, white, speed, LEDcount);
        break;
      case Strobe: // switching between solid color and full off at high speed
        m_toAnimate = new StrobeAnimation(red, green, blue, white, speed, LEDcount);
        break;
      case Twinkle: // random leds turning on and off with certain color
        m_toAnimate =
            new TwinkleAnimation(red, green, blue, white, speed, LEDcount, TwinklePercent.Percent6);
        break;
      case TwinkleOff: // twinkle in reverse
        m_toAnimate =
            new TwinkleOffAnimation(
                red, green, blue, white, speed, LEDcount, TwinkleOffPercent.Percent100);
        break;
      case Solid:
        this.red = red;
        this.green = green;
        this.blue = blue;
        m_toAnimate = null;
        break;
      default:
        System.out.println("Incorrect animation type provided to changeAnimation() method");
        break;
    }
  }
  /**
   * @param state the dominant robot state that LEDs will express
   */
  // will set LEDs a coordinated color for an action !TBD!
  public void expressState(LED_STATE state) {
    if (state != currentRobotState) {
      switch (state) {
        case DISABLED: // Solid red
          setPattern(255, 0, 0, 0, 0, ANIMATION_TYPE.Solid);
          break;
        case INITIALIZED:
          setPattern(0, 255, 0, 0, 0, ANIMATION_TYPE.Twinkle);
          break;
        case ENABLED: // Solid green
          setPattern(0, 255, 0, 0, 0, ANIMATION_TYPE.Solid);
          break;
        case INTAKING: // Solid blue
          setPattern(0, 0, 255, 0, 0, ANIMATION_TYPE.Solid);
          break;
        case CONE_BUTTON: // Solid Yellow
          setPattern(150, 120, 0, 0, 0, ANIMATION_TYPE.Solid);
          break;
        case CUBE_BUTTON: // Solid purple
          setPattern(128, 0, 128, 0, 0, ANIMATION_TYPE.Solid);
          break;
        case ELEVATING:
          setPattern(0, 0, 255, 0, 0, ANIMATION_TYPE.ColorFlow);
          break;
        case CHARGING_STATION:
          setPattern(125, 125, 125, 125, 0, ANIMATION_TYPE.Rainbow);
          break;
        case SCORING: // Flashing white
          setPattern(0, 0, 0, 255, 1, ANIMATION_TYPE.Strobe);
          break;
        case LOCKED_ON: // Flashing Green
          setPattern(0, 255, 0, 0, 1, ANIMATION_TYPE.Strobe);
          break;
        default:
          break;
      }
      currentRobotState = state;
    }
  }

  @Override
  public void periodic() {
    // TODO: Fix this from flashing
    // if (DriverStation.isDisabled()) {
    //   currentRobotState = robotState.DISABLED;
    //   red = 255;
    //   green = 0;
    //   blue = 0;
    //   m_toAnimate = null;
    // }

    // null indicates that the animation is "Solid"
    if (m_toAnimate == null) {
      m_candle.setLEDs(red, green, blue, 0, 0, LEDcount); // setting all LEDs to color
    } else {
      m_candle.animate(m_toAnimate); // setting the candle animation to m_animation if not null
    }
    SmartDashboard.putString("LED Mode", currentRobotState.toString());
    // ledStatePub.set(currentRobotState.toString());
    // the code below was printing out just LED Mode over and over again in the Led tab for some
    // reason but the code above does show the current state
    //   Shuffleboard.getTab("Controls")
    // .add("LED Mode", currentRobotState.toString());

  }

  public INTAKING_STATES getPieceIntent() {
    return pieceIntent;
  }

  public void setPieceIntent(INTAKING_STATES type) {
    pieceIntent = type;
  }

  @Override
  public void close() throws Exception {}
}
