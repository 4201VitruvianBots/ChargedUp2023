package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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

public class ElevatorTimerTest extends SequentialCommandGroup {

  public ElevatorTimerTest(
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Wrist wrist,
      Intake intake,
      Vision vision,
      Elevator elevator,
      StateHandler stateHandler) {

    addCommands(
        new RepeatCommand(
            new SequentialCommandGroup(

                /** Brings elevator & wrist to High Pulls up cone */
                new ParallelCommandGroup(
                    new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.SCORE_HIGH)
                        .withTimeout(WAIT.SCOREHIGH.get()),
                    new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.HOLDING_CONE)
                        .withTimeout(WAIT.SCOREHIGH.get())),
                /** Outakes cone */
                new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.SCORING_CONE)
                    .withTimeout(WAIT.SCORECONE.get()),
                new WaitCommand(WAIT.SCORETIME.get()),
                new PrintCommand("SCORE"),
                new WaitCommand(WAIT.SCORECONE.get()),
                /** Stows Wrist, Elevator, and Stops intake */
                new ParallelCommandGroup(
                    new AutoSetSetpoint(stateHandler, elevator, wrist, SETPOINT.STOWED)
                        .withTimeout(WAIT.STOWHIGHFAST.get()),
                    new AutoSetIntakeSetpoint(intake, INTAKE_SPEEDS.STOP)
                        .withTimeout(WAIT.STOWHIGHFAST.get())),
                /** Runs Path with Intaking cube during */
                new PrintCommand("Driving Driving Driving Driving Driving Driving Driving "),
                new WaitCommand(1))));
  }
}
