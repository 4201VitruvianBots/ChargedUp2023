// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.statehandler;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StateHandler;

public class Runstatehandlertestmode extends CommandBase {
  /** Creates a new Runstatehandlertestmode. */
  private final StateHandler m_stateHandler;

  private DoubleSubscriber Low, Mid, High;
  private double testLow, testMid, testHigh;

  public Runstatehandlertestmode(StateHandler stateHandler) {
    m_stateHandler = stateHandler;
    addRequirements(m_stateHandler);

    NetworkTable stateHandlerNtTab =
        NetworkTableInstance.getDefault()
            .getTable("Shuffleboard")
            .getSubTable("StateHandlerTestMode");
  }

  // Called when the command is initially scheduled.
  //
  //                                            Rizz
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriverStation.reportWarning("USING Statehandler TEST MODE!", false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
