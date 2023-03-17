package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEDRIVE;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  private final Timer m_timer = new Timer();
  private boolean timerStart = false;
  private double timestamp = 0;

  SwerveModuleState[] states;

  PIDController outputCalculator = new PIDController(0.02, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public AutoBalance(SwerveDrive swerveDriveSubsystem) {
    m_swerveDrive = swerveDriveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
    outputCalculator.setSetpoint(0);
    outputCalculator.setTolerance(1);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_swerveDrive.getRollDegrees() + m_swerveDrive.getRollOffset()) > 2.5) {

      double output = outputCalculator.calculate(-m_swerveDrive.getRollDegrees());
      // TODO; set a way to initiallze pitch to 0

      states =
          new SwerveModuleState[] {
            new SwerveModuleState(output, Rotation2d.fromDegrees(90)),
            new SwerveModuleState(output, Rotation2d.fromDegrees(90)),
            new SwerveModuleState(output, Rotation2d.fromDegrees(90)),
            new SwerveModuleState(output, Rotation2d.fromDegrees(90)),
          };

      m_swerveDrive.setSwerveModuleStates(states, false);
    }
    if (Math.abs(m_swerveDrive.getRollDegrees() - m_swerveDrive.getRollOffset()) < 2.5
        && !timerStart
        && timestamp != 0) {
      timerStart = true;
      timestamp = m_timer.get();
    } else if (timerStart
        && Math.abs(m_swerveDrive.getHeadingDegrees() - m_swerveDrive.getRollOffset()) >= 2.5) {
      timerStart = false;
      timestamp = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(
              SWERVEDRIVE.kMaxSpeedMetersPerSecond * 0.011, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(
              SWERVEDRIVE.kMaxSpeedMetersPerSecond * 0.011, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(
              SWERVEDRIVE.kMaxSpeedMetersPerSecond * 0.011, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(
              SWERVEDRIVE.kMaxSpeedMetersPerSecond * 0.011, Rotation2d.fromDegrees(45)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() - timestamp > 2;
  }
}
