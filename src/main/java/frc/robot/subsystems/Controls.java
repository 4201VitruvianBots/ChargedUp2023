package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Controls extends SubsystemBase {
  StringPublisher allianceString;
  BooleanPublisher allianceBoolean;

  private boolean isInit;
  private DriverStation.Alliance allianceColor = DriverStation.Alliance.Invalid;

  public Controls() {
    initSmartDashboard();
    isInit = false;
  }

  /**
   * Returns the robot's current alliance color
   *
   * @return Returns the current alliance color.
   */
  public DriverStation.Alliance getAllianceColor() {
    return allianceColor;
  }

  /**
   * Returns true when the alliance color is not Blue`
   *
   * @return Returns the current alliance color.
   */
  public boolean getAllianceColorBoolean() {
    return getAllianceColor() != DriverStation.Alliance.Blue;
  }

  public void setPDHChannel(boolean on) {
    // pdh.setSwitchableChannel(on);
  }

  public boolean getInitState() {
    return isInit;
  }

  public void setInitState(boolean init) {
    isInit = init;
  }

  /** Initializes values on SmartDashboard */
  private void initSmartDashboard() {
    //    Shuffleboard.getTab("SmartDashboard")
    //            .add("Alliance", getAllianceColorBoolean())
    //            .withWidget(BuiltInWidgets.kBooleanBox)
    //            .withProperties(Map.of("Color when true", "#FF0000", "Color when false",
    // "#0000FF"));
    var controlsTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Controls");
    allianceString = controlsTab.getStringTopic("alliance_string").publish();
    allianceBoolean = controlsTab.getBooleanTopic("Alliance").publish();
    controlsTab.getStringTopic("Robot Name").publish().set(Constants.robotName);
  }

  public void updateAllianceColor() {
    allianceColor = DriverStation.getAlliance();
  }

  /** Sends values to SmartDashboard */
  private void updateSmartDashboard() {
    // SmartDashboard.putBoolean("Alliance", getAllianceColorBoolean());
    allianceString.set(getAllianceColor().toString());
    allianceBoolean.set(getAllianceColorBoolean());
    //    System.out.println("Alliance Color: " + getAllianceColor().toString());
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      updateAllianceColor();
    }
    // This method will be called once per scheduler run
    updateSmartDashboard();
    //    System.out.println("Test1");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    //    System.out.println("Test2");
  }
}
