package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.DoubleSupplier;

public class SetSwerveChassisSpeedRelative extends CommandBase {
  private final SwerveDrive m_swerveDrive;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  public SetSwerveChassisSpeedRelative(
      SwerveDrive swerveDrive,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput) {
    m_swerveDrive = swerveDrive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var chassisSpeeds =
        new ChassisSpeeds(
            m_throttleInput.getAsDouble(),
            m_strafeInput.getAsDouble(),
            m_rotationInput.getAsDouble());
    var states = SWERVE_DRIVE.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

    m_swerveDrive.setSwerveModuleStates(states, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
