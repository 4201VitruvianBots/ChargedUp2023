/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.SwerveDrive;

/** Sets the robot's position */
public class SetSwerveOdometry extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  private final FieldSim m_fieldSim;

  private Pose2d m_pose2d;

  /**
   * Sets the robot's position
   *
   * @param swerveDrive Swerve's odometry is set
   * @param pose2d position to set odometry to
   */
  public SetSwerveOdometry(SwerveDrive swerveDrive, Pose2d pose2d) {
    this(swerveDrive, pose2d, null);
  }

  /**
   * Sets the robot's position
   *
   * @param swerveDrive Swerve's odometry is set
   * @param pose2d position to set odometry to
   * @param fieldSim fieldSim to set robot's position if we're simulating the robot
   */
  public SetSwerveOdometry(SwerveDrive swerveDrive, Pose2d pose2d, FieldSim fieldSim) {
    if (RobotBase.isSimulation() && fieldSim == null)
      System.out.println(
          "SetOdometry Command Error: Robot is in Simulation, but you did not add FieldSim to the argument");

    m_swerveDrive = swerveDrive;
    m_fieldSim = fieldSim;
    m_pose2d = pose2d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pose2d = SimConstants.pathPlannerFlip(m_pose2d);

    if (RobotBase.isSimulation()) m_fieldSim.resetRobotPose(m_pose2d);
    m_swerveDrive.setOdometry(m_pose2d);
    //    SmartDashboard.putNumber("SwerveInitialPositionX", m_pose2d.getX());
    //    SmartDashboard.putNumber("SwerveInitialPositionY", m_pose2d.getY());
    //    SmartDashboard.putNumber("SwerveInitialPositionRotation",
    // m_pose2d.getRotation().getDegrees());
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
