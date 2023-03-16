package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

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
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
    outputCalculator.setSetpoint(0);
    outputCalculator.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_swerveDrive.getPitchDegrees() + 2.460938) > 3
        || (m_swerveDrive.getPitchDegrees() + 2.460938) < -3) {

      double output = outputCalculator.calculate(m_swerveDrive.getPitchDegrees());
      // TODO; set a way to initiallze pitch to 0

      states =
          new SwerveModuleState[] {
            new SwerveModuleState(output * 1.1, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(output * 1.1, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(output * 1.1, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(output * 1.1, Rotation2d.fromDegrees(0)),
          };

      m_swerveDrive.setSwerveModuleStates(states, false);
    } else if ((m_swerveDrive.getPitchDegrees() + 2.460938) <= 3) {
      SmartDashboard.putNumber("moduleangle", m_swerveDrive.getPitchDegrees());
      System.out.print("hourrah");
      states =
          new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          };

      m_swerveDrive.setSwerveModuleStates(states, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        };
    m_swerveDrive.setSwerveModuleStates(states, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
