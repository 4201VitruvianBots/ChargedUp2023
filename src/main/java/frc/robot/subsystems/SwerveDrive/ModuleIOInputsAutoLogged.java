package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleIOInputsAutoLogged extends ModuleIO.ModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DrivePositionRad", drivePositionRad);
    table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
    table.put("DriveVelocityFilteredRadPerSec", driveVelocityFilteredRadPerSec);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("DriveCurrentAmps", driveCurrentAmps);
    table.put("DriveTempCelcius", driveTempCelcius);
    table.put("TurnAbsolutePositionRad", turnAbsolutePositionRad);
    table.put("TurnPositionRad", turnPositionRad);
    table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec);
    table.put("TurnAppliedVolts", turnAppliedVolts);
    table.put("TurnCurrentAmps", turnCurrentAmps);
    table.put("TurnTempCelcius", turnTempCelcius);
  }

  @Override
  public void fromLog(LogTable table) {
    drivePositionRad = table.getDouble("DrivePositionRad", drivePositionRad);
    driveVelocityRadPerSec = table.getDouble("DriveVelocityRadPerSec", driveVelocityRadPerSec);
    driveVelocityFilteredRadPerSec = table.getDouble("DriveVelocityFilteredRadPerSec", driveVelocityFilteredRadPerSec);
    driveAppliedVolts = table.getDouble("DriveAppliedVolts", driveAppliedVolts);
    driveCurrentAmps = table.getDoubleArray("DriveCurrentAmps", driveCurrentAmps);
    driveTempCelcius = table.getDoubleArray("DriveTempCelcius", driveTempCelcius);
    turnAbsolutePositionRad = table.getDouble("TurnAbsolutePositionRad", turnAbsolutePositionRad);
    turnPositionRad = table.getDouble("TurnPositionRad", turnPositionRad);
    turnVelocityRadPerSec = table.getDouble("TurnVelocityRadPerSec", turnVelocityRadPerSec);
    turnAppliedVolts = table.getDouble("TurnAppliedVolts", turnAppliedVolts);
    turnCurrentAmps = table.getDoubleArray("TurnCurrentAmps", turnCurrentAmps);
    turnTempCelcius = table.getDoubleArray("TurnTempCelcius", turnTempCelcius);
  }

  public ModuleIOInputsAutoLogged clone() {
    ModuleIOInputsAutoLogged copy = new ModuleIOInputsAutoLogged();
    copy.drivePositionRad = this.drivePositionRad;
    copy.driveVelocityRadPerSec = this.driveVelocityRadPerSec;
    copy.driveVelocityFilteredRadPerSec = this.driveVelocityFilteredRadPerSec;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.driveCurrentAmps = this.driveCurrentAmps.clone();
    copy.driveTempCelcius = this.driveTempCelcius.clone();
    copy.turnAbsolutePositionRad = this.turnAbsolutePositionRad;
    copy.turnPositionRad = this.turnPositionRad;
    copy.turnVelocityRadPerSec = this.turnVelocityRadPerSec;
    copy.turnAppliedVolts = this.turnAppliedVolts;
    copy.turnCurrentAmps = this.turnCurrentAmps.clone();
    copy.turnTempCelcius = this.turnTempCelcius.clone();
    return copy;
  }
}
