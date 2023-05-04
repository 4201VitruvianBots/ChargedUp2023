package frc.robot.subsystems.SwerveDrive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("PositionRad", positionRad);
    table.put("VelocityRadPerSec", velocityRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.getBoolean("Connected", connected);
    positionRad = table.getDouble("PositionRad", positionRad);
    velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.positionRad = this.positionRad;
    copy.velocityRadPerSec = this.velocityRadPerSec;
    return copy;
  }
}
