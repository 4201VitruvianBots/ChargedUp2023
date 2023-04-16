package frc.robot.commands.auto;

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
                new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH_CUBE)
                    .withTimeout(WAIT.SCORE_HIGH_CUBE.get()),
                new AutoSetIntakeSetpoint(intake, INTAKE_STATE.HOLDING_CUBE, vision, swerveDrive)
                    .withTimeout(WAIT.SCORE_HIGH_CUBE.get())),
            /** Outakes cone */
            new WaitCommand(WAIT.WAIT_TO_PLACE_CUBE.get()),
            new AutoSetIntakeSetpoint(intake, INTAKE_STATE.SCORING_CUBE, vision, swerveDrive)
                .withTimeout(WAIT.SCORING_CUBE.get()),
            new PrintCommand("SCORE"),
            new WaitCommand(WAIT.SCORING_CUBE.get()),
            /** Stows Wrist, Elevator, and Stops intake */
            new ParallelCommandGroup(
                new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED)
                    .withTimeout(WAIT.STOW_HIGH_CUBE.get()),
                new AutoSetIntakeSetpoint(intake, INTAKE_STATE.NONE, vision, swerveDrive)
                    .withTimeout(WAIT.STOW_HIGH_CUBE.get())),
            /** Runs Path with Intaking cube during */
            new PrintCommand("DRIVING"),
            new WaitCommand(1)));
  }
}
