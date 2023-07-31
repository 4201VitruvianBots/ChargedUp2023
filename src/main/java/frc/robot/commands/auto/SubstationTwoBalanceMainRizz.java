// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AUTO.WAIT;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.STATE_HANDLER.SETPOINT;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.Constants.VISION.PIPELINE;
import frc.robot.commands.DelayedInterruptingCommand;
import frc.robot.commands.intake.AutoSetIntakeSetpoint;
import frc.robot.commands.statehandler.AutoSetSetpoint;
import frc.robot.commands.statehandler.SetSetpoint;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.DriveForwardWithVisionInput;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StateHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.TrajectoryUtils;

public class SubstationTwoBalanceMainRizz extends  SequentialCommandGroup {

  public SubstationTwoBalanceMainRizz( SwerveDrive swerveDrive ,String pathName,
  FieldSim fieldSim,
  Wrist wrist,
  Intake intake,
  Vision vision,
  Elevator elevator,
  StateHandler stateHandler){
      // Dependencies like SwerveDrive, Intake etc) 
     // Part 1 constraints
PathConstraints part1Constraints = new PathConstraints(maxVel, maxAccel1);
List<PathPlannerTrajectory> part1Trajectories = TrajectoryUtils.readTrajectory(pathName, part1Constraints);

// Part 2 constraints  
PathConstraints part2Constraints = new PathConstraints(maxVel2, maxAccel2);
List<PathPlannerTrajectory> part2Trajectories = TrajectoryUtils.readTrajectory(pathName, part2Constraints);

// Generate commands
List<SwerveControllerCommand> part1Commands = generateSwerveCommands(swerveDrive, part1Trajectories); 
List<SwerveControllerCommand> part2Commands = generateSwerveCommands(swerveDrive, part2Trajectories);
      
    addCommands(

      new SubstationTwoBalancePart1(swerveCommands, swerveDrive, fieldSim, wrist, intake, vision, elevator, stateHandler),
      
      new SubstationTwoBalancePart2(swerveCommands,  fieldSim, wrist, intake, vision, elevator, stateHandler ),
      
      new SubstationTwoBalancePart3(swerveCommands , fieldSim, wrist, intake, vision, elevator, stateHandler )
      
    );

  }

  private List<Trajectory> generateTrajectories() {
    // trajectory generation
  }

  List<SwerveControllerCommand> generateSwerveCommands(
    SwerveDrive swerveDrive, List<PathPlannerTrajectory> trajectories) {
  
  return TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, trajectories);

}
  
    // swerve command generation
    TrajectoryUtils.generatePPSwerveControllerCommand(swerveDrive, m_trajectories);
  }



