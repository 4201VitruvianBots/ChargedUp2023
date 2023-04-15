package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE.INTAKE_SPEEDS;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.commands.intake.AutoSetIntakeSetpoint;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class LimeLightTest extends SequentialCommandGroup {
  public LimeLightTest(
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    addCommands(
        /** Setting Up Auto Zeros robot to path flips path if necessary */
        new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.INTAKING_LOW_CUBE),
        new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.INTAKING_CUBE, vision, swerveDrive));
  }
}
