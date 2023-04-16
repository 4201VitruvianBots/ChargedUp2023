package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVE_DRIVE;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class DriveForwardWithVisionInput extends CommandBase {
  private final SwerveDrive m_swerveDrive;
  private final Vision m_vision;

  private final DoubleSupplier m_throttleInput;

  private final PIDController strafePIDController =
      new PIDController(0.1, SWERVE_DRIVE.kI_Y, SWERVE_DRIVE.kD_Y);

  public DriveForwardWithVisionInput(
      SwerveDrive swerveDrive, Vision vision, DoubleSupplier throttleInput) {
    m_swerveDrive = swerveDrive;
    m_vision = vision;
    m_throttleInput = throttleInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafeOutput = 0;
    if (m_vision.getValidTarget(CAMERA_SERVER.INTAKE))
      strafeOutput = strafePIDController.calculate(-m_vision.getTargetXAngle(CAMERA_SERVER.INTAKE));

    var chassisSpeeds = new ChassisSpeeds(m_throttleInput.getAsDouble(), strafeOutput, 0);
    var states = SWERVE_DRIVE.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

    m_swerveDrive.setSwerveModuleStates(states, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
