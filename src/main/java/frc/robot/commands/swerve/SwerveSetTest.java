package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SwerveSetTest extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  private final Timer m_timer = new Timer();
  private boolean timerStart = false;

  SwerveModuleState[] states;

  private double m_output = 2;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SwerveSetTest(SwerveDrive swerveDriveSubsystem) {
    m_swerveDrive = swerveDriveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int i = 0; i < 3; i++) {
      if (m_swerveDrive.getPoseMeters().getY() <= 2) {
        isFinished();
      }

      states =
          new SwerveModuleState[] {
            new SwerveModuleState(m_output, Rotation2d.fromDegrees(90 * i)),
            new SwerveModuleState(m_output, Rotation2d.fromDegrees(90 * i)),
            new SwerveModuleState(m_output, Rotation2d.fromDegrees(90 * i)),
            new SwerveModuleState(m_output, Rotation2d.fromDegrees(90 * i)),
          };

      m_swerveDrive.setSwerveModuleStates(states, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        };
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  public double getOutput() {
    return m_output;
  }
}
