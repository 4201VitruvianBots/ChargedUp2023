package frc.robot.commands.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/** Sets the drivetrain to neutral (coast/brake) */
public class SetSwerveNeutralMode extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  private final NeutralMode m_mode;

  /**
   * Sets the drivetrain neutral mode (coast/brake).
   *
   * @param swerveDrive The driveTrain used by this command.
   * @param mode {@link NeutralMode}: COAST, BRAKE, or HALF_BRAKE.
   */
  public SetSwerveNeutralMode(SwerveDrive swerveDrive, NeutralMode mode) {
    m_swerveDrive = swerveDrive;
    m_mode = mode;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.setNeutralMode(m_mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
