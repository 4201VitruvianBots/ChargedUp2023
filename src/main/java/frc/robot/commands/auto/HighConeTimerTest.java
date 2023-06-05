package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AUTO.WAIT;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.commands.intake.AutoSetIntakeSetpoint;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.wrist.Wrist;

public class HighConeTimerTest extends SequentialCommandGroup {

  public HighConeTimerTest(
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    Timer timer = new Timer();

    addCommands(
        new InstantCommand(() -> timer.reset()),
        new InstantCommand(() -> timer.start()),
        new SequentialCommandGroup(

            /** Brings elevator & wrist to High Pulls up cone */
            new ParallelCommandGroup(
                new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH_CONE)
                    .withTimeout(WAIT.SCORE_HIGH_CONE.get()),
                new AutoSetIntakeSetpoint(intake, INTAKE_STATE.HOLDING_CONE, vision, swerveDrive)
                    .withTimeout(WAIT.SCORE_HIGH_CONE.get())),

            /** Outakes cone */
            new WaitCommand(WAIT.WAIT_TO_PLACE_CONE.get()),
            new PrintCommand(String.format("Object Scoring starts at: %f", timer.get())),
            new AutoSetIntakeSetpoint(intake, INTAKE_STATE.SCORING_CONE, vision, swerveDrive)
                .withTimeout(WAIT.SCORING_CONE.get()),
            new WaitCommand(WAIT.SCORING_CONE.get()),
            new PrintCommand(String.format("Object Scoring finishes at: %f", timer.get())),
            /** Stows Wrist, Elevator, and Stops intake */
            new ParallelCommandGroup(
                new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED)
                    .withTimeout(WAIT.STOW_HIGH_CONE.get()),
                new AutoSetIntakeSetpoint(intake, INTAKE_STATE.NONE, vision, swerveDrive)
                    .withTimeout(WAIT.STOW_HIGH_CONE.get())),
            new WaitCommand(WAIT.STOW_HIGH_CONE.get()),

            /** Runs Path with Intaking cube during */
            new PrintCommand(String.format("Command Ends at: %f", timer.get()))));
  }
}
