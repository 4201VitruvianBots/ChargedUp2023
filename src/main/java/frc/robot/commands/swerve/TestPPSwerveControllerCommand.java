package frc.robot.commands.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.Supplier;

public class TestPPSwerveControllerCommand extends PPSwerveControllerCommand {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory m_trajectory;
  private final Supplier<Pose2d> m_poseSupplier;
  private final SwerveDriveKinematics m_kinematics;
  private final PPHolonomicDriveController m_controller;
  /** Creates a new FollowTrajectory. */
  public TestPPSwerveControllerCommand(SwerveDrive swerveDrive, PathPlannerTrajectory trajectory) {
    super(
        trajectory,
        swerveDrive::getPoseMeters,
        Constants.SwerveDrive.kSwerveKinematics,
        swerveDrive.getXPidController(),
        swerveDrive.getYPidController(),
        swerveDrive.getThetaPidController(),
        swerveDrive::setSwerveModuleStatesAuto,
        false,
        swerveDrive);
    m_trajectory = trajectory;
    m_poseSupplier = swerveDrive::getPoseMeters;
    m_kinematics = Constants.SwerveDrive.kSwerveKinematics;
    m_controller =
        new PPHolonomicDriveController(
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController());
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    this.timer.reset();
    this.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    double currentTime = this.timer.get();
    PathPlannerTrajectory.PathPlannerState desiredState =
        (PathPlannerTrajectory.PathPlannerState) m_trajectory.sample(currentTime);

    Pose2d currentPose = this.m_poseSupplier.get();

    ChassisSpeeds targetChassisSpeeds = this.m_controller.calculate(currentPose, desiredState);
    SwerveModuleState[] targetModuleStates =
        this.m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
    System.out.println("Current Pose " + currentPose);
    System.out.println("Desired Translation " + desiredState.poseMeters);
    System.out.println("Desired Rotation " + desiredState.holonomicRotation.getDegrees());
    System.out.println("Target Speeds " + targetChassisSpeeds);
    for (int i = 0; i < targetModuleStates.length; i++)
      System.out.println("Module " + i + " Target State " + targetModuleStates[i]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
