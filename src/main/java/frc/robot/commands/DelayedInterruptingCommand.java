package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;

/**
 * Runs a command until a condition is met, then interrupts it to run another command. If the first
 * command finishes, this command will end without running the other command.
 */
// Knock knock
// Who's there?
// Interrupting command
// Interrupting comma-
public class DelayedInterruptingCommand extends CommandBase {
  private final Command m_interruptible;
  private final Command m_interrupt;
  private Command m_selectedCommand;

  private final double m_delay;
  private final BooleanSupplier m_condition;
  private boolean hasInterrupted = false;

  private final Timer m_timer = new Timer();

  /**
   * Runs a command until a condition is met, then interrupts it to run another command
   *
   * @param interruptible The command to run initially that will be interrupted
   * @param interrupt The command that interrupts the first command
   * @param condition When to interrupt the first command
   */
  public DelayedInterruptingCommand(
      Command interruptible, Command interrupt, double delay, BooleanSupplier condition) {
    m_interruptible = interruptible;
    m_interrupt = interrupt;
    m_delay = delay;
    m_condition = condition;
    m_requirements.addAll(interruptible.getRequirements());
    m_requirements.addAll(interrupt.getRequirements());
  }

  @Override
  public void initialize() {
    hasInterrupted = false;
    m_selectedCommand = m_interruptible;
    m_selectedCommand.initialize();
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    if (m_condition.getAsBoolean() && !hasInterrupted && m_timer.hasElapsed(m_delay)) {
      hasInterrupted = true;
      m_selectedCommand.end(true);
      m_selectedCommand = m_interrupt;
      m_selectedCommand.initialize();
    }
    m_selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_selectedCommand.isFinished();
  }
}
