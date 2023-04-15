package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AUTOTIMES.WAIT;
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

public class HighCubeTimerTest extends SequentialCommandGroup {

  public HighCubeTimerTest(
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    addCommands(
        new SequentialCommandGroup(

            /** Brings elevator & wrist to High Pulls up cone */
            new ParallelCommandGroup(
                new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH_CONE)
                    .withTimeout(WAIT.SCORE_HIGH_CONE.get()),
                new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.HOLDING_CONE, vision, swerveDrive)
                    .withTimeout(WAIT.SCORE_HIGH_CONE.get())),
            /** Outakes cone */
            new WaitCommand(WAIT.WAIT_TO_PLACE_CONE.get()),
            new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.SCORING_CONE, vision, swerveDrive)
                .withTimeout(WAIT.SCORING_CONE.get()),
            new PrintCommand("SCORE"),
            new WaitCommand(WAIT.SCORING_CONE.get()),
            /** Stows Wrist, Elevator, and Stops intake */
            new ParallelCommandGroup(
                new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED)
                    .withTimeout(WAIT.STOW_HIGH_CONE.get()),
                new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.STOP, vision, swerveDrive)
                    .withTimeout(WAIT.STOW_HIGH_CONE.get())),
            /** Runs Path with Intaking cube during */
            new PrintCommand("DRIVING"),
            new WaitCommand(1)));
  }
}
