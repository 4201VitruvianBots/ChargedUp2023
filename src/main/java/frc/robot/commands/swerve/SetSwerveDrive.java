package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.DoubleSupplier;

public class SetSwerveDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDrive The subsystem used by this command.
   */
  public SetSwerveDrive(
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
    double throttle =
        MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()), 0.05)
            * Math.signum(m_throttleInput.getAsDouble());
    double strafe =
        MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()), 0.05)
            * Math.signum(m_strafeInput.getAsDouble());
    double rotation =
        MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()), 0.05)
            * Math.signum(m_rotationInput.getAsDouble());

    //    if (DriverStation.isFMSAttached()
    //        && Controls.getAllianceColor() == DriverStation.Alliance.Red) {
    //      throttle *= -1;
    //      strafe *= -1;
    //    }

    m_swerveDrive.drive(throttle, strafe, rotation, true, false);
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
