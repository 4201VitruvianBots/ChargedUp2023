package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


//creates LED subsystem
public class LED extends SubsystemBase {

//rgb spectrum
int red = 0;
int green = 0;
int blue = 0;

//will creates LED strip
public LED (Controls controls) {

}

//sets up LED strip
CANdleConfiguration configall = new CANdle();
configAll.striptype = LEDStripType.GRB

//will set LED color and animation type
@param red
@param green
@param blue
@param white
@param speed
@param toChange

//will create LED patterns
public void setPattern() {

}

//will set LEDs a coordinated color for an action
public void expressState(robotState robotState) {
if(state != robotState);
    case Intaking:
        setPattern(red: 0, green: 0, blue: 0, speed: 0, AnimationTypes.Solid);
        break;
    case Elavating:
        setPattern(red: 0, green: 0, blue: 0, speed: 0, AnimationTypes.Solid);
        break;
    case Extending:
        setPattern(red: 0, green: 0, blue: 0, speed: 0, AnimationTypes.Solid);
}

}
