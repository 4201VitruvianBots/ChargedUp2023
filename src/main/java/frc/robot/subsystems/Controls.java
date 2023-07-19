package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("RedundantThrows")
public class Controls extends SubsystemBase implements AutoCloseable {
  private StringPublisher allianceString;
  private BooleanPublisher allianceBoolean;

  private boolean isInit;
  private static DriverStation.Alliance allianceColor = DriverStation.Alliance.Red;

  public Controls() {
    initSmartDashboard();
    isInit = false;
  }

  /**
   * Returns the robot's current alliance color
   *
   * @return Returns the current alliance color.
   */
  public static DriverStation.Alliance getAllianceColor() {
    return allianceColor;
  }

  /**
   * Returns true when the alliance color is not Blue`
   *
   * @return Returns the current alliance color.
   */
  public static boolean getAllianceColorBoolean() {
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
    var controlsTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Controls");
    allianceString = controlsTab.getStringTopic("alliance_string").publish();
    allianceBoolean = controlsTab.getBooleanTopic("Alliance").publish();
    try {
      controlsTab.getStringTopic("Robot Name").publish().set(Constants.robotName);
    } catch (Exception m_ignored) {

    }
  }

  /**
   * Periodically check the DriverStation to get the Alliance color. This mainly runs when the robot
   * is disabled to avoid a bug where the robot tries to get the alliance color before it is
   * connected to a driver station.
   */
  private void updateAllianceColor() {
    allianceColor = DriverStation.getAlliance();
  }

  /** Sends values to SmartDashboard */
  private void updateSmartDashboard() {
    allianceString.set(getAllianceColor().toString());
    allianceBoolean.set(getAllianceColorBoolean());
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation() || (RobotBase.isReal() && DriverStation.isDisabled())) {
      updateAllianceColor();
    }
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
