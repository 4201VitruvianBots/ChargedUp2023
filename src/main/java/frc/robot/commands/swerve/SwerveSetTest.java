package frc.robot.commands.swerve;

import static frc.robot.Constants.AUTO.kAutoBalanceAngleThresholdDegrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  private final Timer m_timer = new Timer();
  private boolean timerStart = false;

  SwerveModuleState[] states;

  private final PIDController outputCalculator = new PIDController(0.0225, 0, 0);
  private double m_output = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public AutoBalance(SwerveDrive swerveDriveSubsystem) {
    m_swerveDrive = swerveDriveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
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
    outputCalculator.setTolerance(3);
    timerStart = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Negative & 90 degrees for Facing Bump Side
    m_output =
        -outputCalculator.calculate(
            m_swerveDrive.getRollDegrees() + m_swerveDrive.getRollOffsetDegrees());

    states =
        new SwerveModuleState[] {
          new SwerveModuleState(m_output, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(m_output, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(m_output, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(m_output, Rotation2d.fromDegrees(90)),
        };

    m_swerveDrive.setSwerveModuleStates(states, false);

    double balanceDeltaDegrees =
        Math.abs(m_swerveDrive.getRollDegrees() + m_swerveDrive.getRollOffsetDegrees());
    if (balanceDeltaDegrees < kAutoBalanceAngleThresholdDegrees && !timerStart) {
      m_timer.reset();
      m_timer.start();
      timerStart = true;
      outputCalculator.setP(0.00025);
    } else if (balanceDeltaDegrees >= kAutoBalanceAngleThresholdDegrees && timerStart) {
      m_timer.reset();
      m_timer.stop();
      timerStart = false;
      outputCalculator.setP(0.001);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(
              SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.011, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(
              SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.011, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(
              SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.011, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(
              SWERVE_DRIVE.kMaxSpeedMetersPerSecond * 0.011, Rotation2d.fromDegrees(45)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(2) && timerStart;
  }

  public double getOutput() {
    return m_output;
  }
}
