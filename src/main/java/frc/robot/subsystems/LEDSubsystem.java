package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.LED;
import frc.robot.Constants.LED.*;
import frc.robot.Constants.STATE_HANDLER.SUPERSTRUCTURE_STATE;

// creates LED subsystem
public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  private final CANdle m_candle = new CANdle(CAN.CANdle); // LED In constants)
  private int red = 0;
  private int green = 0; // setting all LED colors to none: there is no color when robot activates
  private int blue = 0;
  private SUPERSTRUCTURE_STATE currentRobotState = SUPERSTRUCTURE_STATE.STOWED;
  private boolean setSolid;
  private Animation m_toAnimate = null;

  private final StringPublisher ledStatePub;

  // Mechanism2d visualization setup
  public final Mechanism2d m_mech2d = new Mechanism2d(1, 1);
  public final MechanismRoot2d m_root2d = m_mech2d.getRoot("LED", 0.5, 0);
  public final MechanismLigament2d m_ligament2d = m_root2d.append(new MechanismLigament2d("LED", 2, 90));

  // Create LED strip
  public LEDSubsystem(Controls controls) {
    m_candle.configFactoryDefault(); // sets up LED strip
    // sets up LED strip
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true; // sets lights of when the LEDs are activated
    configAll.disableWhenLOS = false; // disables LEDs when there is no signal for control
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar =
        0.5; // 1 is highest we can go we don't want to blind everyone at the event
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
    var nt_instance =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Controls");
    ledStatePub = nt_instance.getStringTopic("LED State").publish();

    // Initialize visualization
    m_ligament2d.setLineWeight(1000); // making the line THICK
    SmartDashboard.putData("LED Sim", m_mech2d);
  }

  /**
   * Will set LED color and animation type
   *
   * @param color the color channel
   * @param white LED intensity
   * @param speed Animation Speed
   * @param toChange AnimationType to use
   */

  // will create LED patterns
  public void setPattern(Color8Bit color, int white, double speed, ANIMATION_TYPE toChange) {
    int red = color.red;
    int green = color.green;
    int blue = color.blue;
    switch (toChange) {
      case ColorFlow: // stripe of color flowing through the LED strip
        m_toAnimate =
            new ColorFlowAnimation(red, green, blue, white, speed, LED.LEDcount, Direction.Forward);
        break;
      case Fire: // red and orange LEDs flaming up and down the LED strip
        m_toAnimate = new FireAnimation(0.5, 0.7, LED.LEDcount, 0.7, 0.5);
        break;
      case Larson: // a line bouncing back and forth with its width determined by size
        m_toAnimate =
            new LarsonAnimation(red, green, blue, white, speed, LED.LEDcount, BounceMode.Front, 7);
        break;
      case Rainbow: // neon cat type beat
        m_toAnimate = new RainbowAnimation(1, speed, LED.LEDcount);
        break;
      case RgbFade: // cycling between red, greed, and blue
        m_toAnimate = new RgbFadeAnimation(1, speed, LED.LEDcount);
        break;
      case SingleFade: // slowly turn all LEDs from solid color to off
        m_toAnimate = new SingleFadeAnimation(red, green, blue, white, speed, LED.LEDcount);
        break;
      case Strobe: // switching between solid color and full off at high speed
        m_toAnimate = new StrobeAnimation(red, green, blue, white, speed, LED.LEDcount);
        break;
      case Twinkle: // random LEDs turning on and off with certain color
        m_toAnimate =
            new TwinkleAnimation(
                red, green, blue, white, speed, LED.LEDcount, TwinklePercent.Percent6);
        break;
      case TwinkleOff: // twinkle in reverse
        m_toAnimate =
            new TwinkleOffAnimation(
                red, green, blue, white, speed, LED.LEDcount, TwinkleOffPercent.Percent100);
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
  public void expressState(SUPERSTRUCTURE_STATE state) {
    if (state != currentRobotState) {
      switch (state) {
        case INTAKE_LOW_CONE:
          setPattern(LED.yellow, 0, 0, ANIMATION_TYPE.Solid);
          break;
        case INTAKE_LOW_CUBE:
          setPattern(LED.purple, 0, 0, ANIMATION_TYPE.Solid);
          break;
        case ALPHA_ZONE:
        case SCORE_LOW_REVERSE:
        case SCORE_LOW:
        case SCORE_LOW_CONE:
        case SCORE_LOW_CUBE:
          setPattern(LED.white, 0, 0, ANIMATION_TYPE.Solid); // Solid White
          break;
        case BETA_ZONE:
        case SCORE_MID:
        case SCORE_MID_CONE:
        case SCORE_MID_CUBE:
          setPattern(LED.blue, 15, 0, ANIMATION_TYPE.Solid); // Solid Blue
          break;
        case GAMMA_ZONE:
        case INTAKE_EXTENDED:
        case SCORE_HIGH:
        case SCORE_HIGH_CONE:
        case SCORE_HIGH_CUBE:
          setPattern(LED.pink, 0, 0, ANIMATION_TYPE.Solid); // Solid Pink
          break;
        case DISABLED:
          setPattern(LED.red, 0, 0, ANIMATION_TYPE.Solid); // Solid Red
          break;
        case ENABLED:
          setPattern(LED.green, 0, 0, ANIMATION_TYPE.Solid); // Solid Green
          break;
        case LOW_BATTERY:
          setPattern(LED.yellow, 0, 1, ANIMATION_TYPE.Strobe); // Flashing Yellow
          break;
        default:
          break;
      }
      currentRobotState = state;
    }
  }

  @Override
  public void simulationPeriodic() {
    m_ligament2d.setColor(new Color8Bit(this.red, this.green, this.blue));
  }

  @Override
  public void periodic() {
    // null indicates that the animation is "Solid"
    if (m_toAnimate == null && !setSolid) {
      setSolid = true;
      m_candle.setLEDs(red, green, blue, 0, 0, LED.LEDcount); // setting all LEDs to color
    } else {
      setSolid = false;
      m_candle.animate(m_toAnimate); // setting the candle animation to m_animation if not null
    }

    if (DriverStation.isDisabled()) {
      if (RobotController.getBatteryVoltage()
          < 10) { // calling battery to let driver know that it is low
        expressState(SUPERSTRUCTURE_STATE.LOW_BATTERY);
      }
    }

    SmartDashboard.putString("LED Mode", currentRobotState.toString());
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
