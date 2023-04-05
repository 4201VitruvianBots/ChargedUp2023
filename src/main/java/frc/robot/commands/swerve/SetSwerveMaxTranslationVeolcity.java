package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

/** Sets the drivetrain to neutral (coast/brake) */
public class SetSwerveMaxTranslationVeolcity extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  private final double m_velocityMps;
  /**
   * Sets the drivetrain neutral mode (coast/brake).
   *
   * @param driveTrain The driveTrain used by this command.
   * @param mode {@link DriveTrainNeutralMode}: COAST, BRAKE, or HALF_BRAKE.
   */
  public SetSwerveMaxTranslationVeolcity(SwerveDrive swerveDrive, double velocityMps) {
    m_swerveDrive = swerveDrive;
    m_velocityMps = velocityMps;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.setMaxVelocity(m_velocityMps);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.setMaxVelocity(Constants.SWERVE_DRIVE.kMaxSpeedMetersPerSecond);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
