// package frc.robot.commands.auto;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.swerve.SetSwerveNeutralMode;
// import frc.robot.commands.swerve.SetSwerveOdometry;
// import frc.robot.simulation.FieldSim;
// import frc.robot.subsystems.SwerveDrive;

// public class DriveTest extends SequentialCommandGroup {
//   public DriveTest(SwerveDrive swerveDrive, FieldSim fieldSim) {
//     SwerveModuleState[] states =
//         new SwerveModuleState[] {
//           new SwerveModuleState(0.1, new Rotation2d()),
//           new SwerveModuleState(0.1, new Rotation2d()),
//           new SwerveModuleState(0.1, new Rotation2d()),
//           new SwerveModuleState(0.1, new Rotation2d())
//         };
//     addCommands(
//         new SetSwerveOdometry(
//             swerveDrive, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)), fieldSim),
//         new RunCommand(() -> swerveDrive.setSwerveModuleStatesAuto(states)),
//         new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
//             .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
//   }
// }
