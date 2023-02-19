// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Vision.CAMERA_POSITION;
import java.util.stream.DoubleStream;

public class Vision extends SubsystemBase {

  private final SwerveDrive m_swerveDrive;
  private final Controls m_controls;

  private final NetworkTable intake;
  private final NetworkTable outtake;
  private final NetworkTable m_leftLocalizer;
  private final NetworkTable m_rightLocalizer;

  private final DoubleArrayPublisher m_leftLocalizerPositionPub;
  private final DoubleArrayPublisher m_rightLocalizerPositionPub;

  private DoubleLogEntry limelightTargetValid;
  private DoubleLogEntry leftLocalizerTargetValid;

  Pose2d defaultPose = new Pose2d(-5, -5, new Rotation2d());

  double[] defaultDoubleArray = {0, 0, 0, 0, 0, 0};

  int[] tagIds = new int[10];
  double[] robotPosX = new double[10];
  double[] robotPosY = new double[10];
  double[] robotPosYaw = new double[10];

  double[] tagPosX = new double[10];
  double[] tagPosY = new double[10];

  public Vision(SwerveDrive swerveDrive, DataLog logger, Controls controls) {
    m_swerveDrive = swerveDrive;
    m_controls = controls;

    intake = NetworkTableInstance.getDefault().getTable("limelight");
    outtake = NetworkTableInstance.getDefault().getTable("limelight");
    m_leftLocalizer = NetworkTableInstance.getDefault().getTable("lLocalizer");
    m_rightLocalizer = NetworkTableInstance.getDefault().getTable("rLocalizer");

    PortForwarder.add(5800, Constants.Vision.SERVER_IPS.RIGHT_LOCALIZER.toString(), 5800);
    PortForwarder.add(5801, Constants.Vision.SERVER_IPS.RIGHT_LOCALIZER.toString(), 5801);
    PortForwarder.add(5802, Constants.Vision.SERVER_IPS.RIGHT_LOCALIZER.toString(), 5802);
    PortForwarder.add(5803, Constants.Vision.SERVER_IPS.RIGHT_LOCALIZER.toString(), 5803);
    PortForwarder.add(5804, Constants.Vision.SERVER_IPS.RIGHT_LOCALIZER.toString(), 5804);
    PortForwarder.add(5805, Constants.Vision.SERVER_IPS.RIGHT_LOCALIZER.toString(), 5805);

    limelightTargetValid = new DoubleLogEntry(logger, "/vision/limelight_tv");
    leftLocalizerTargetValid = new DoubleLogEntry(logger, "/vision/fLocalizer_tv");


    m_leftLocalizerPositionPub = NetworkTableInstance.getDefault()
        .getTable("lLocalizer")
        .getDoubleArrayTopic("camToRobotT3D")
        .publish();

    m_rightLocalizerPositionPub = NetworkTableInstance.getDefault()
        .getTable("rLocalizer")
        .getDoubleArrayTopic("camToRobotT3D")
        .publish();
  }

  /**
   * Given a camera, return a boolean value based on if it sees a target or not.
   *
   * @return true: Camera has a target. false: Camera does not have a target
   */
  public boolean getValidTarget(CAMERA_POSITION position) {
    return getValidTargetType(position) > 0;
  }

  public double getValidTargetType(CAMERA_POSITION position) {
    switch (position) {
      case INTAKE:
        return intake.getEntry("tv").getDouble(0);
      case OUTTAKE:
        return outtake.getEntry("tv").getDouble(0);
      case LEFT_LOCALIZER:
        return m_leftLocalizer.getEntry("tv").getDouble(0);
      case RIGHT_LOCALIZER:
        return m_rightLocalizer.getEntry("tv").getDouble(0);
      default:
        return 0;
    }
  }

  public double[] getAprilTagIds(CAMERA_POSITION position) {
    switch (position) {
      case LEFT_LOCALIZER:
        return m_leftLocalizer.getEntry("tid").getDoubleArray(defaultDoubleArray);
      case RIGHT_LOCALIZER:
        return m_rightLocalizer.getEntry("tid").getDoubleArray(defaultDoubleArray);
      default:
        return defaultDoubleArray;
    }
  }

  public double getTargetXAngle(CAMERA_POSITION position, int index) {
    switch (position) {
      case INTAKE:
        return -intake.getEntry("tx").getDouble(0);
      case OUTTAKE:
        return -outtake.getEntry("tx").getDouble(0);
      default:
        return 0;
    }
  }

  public double getTargetYAngle(CAMERA_POSITION position, int index) {
    switch (position) {
      case INTAKE:
        return intake.getEntry("ty").getDouble(0);
      case OUTTAKE:
        return outtake.getEntry("ty").getDouble(0);
      default:
        return 0;
    }
  }

  public double getCameraLatency(CAMERA_POSITION position) {
    switch (position) {
      case INTAKE:
        return intake.getEntry("tl").getDouble(0);
      case OUTTAKE:
        return outtake.getEntry("tl").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Full JSON dump of targeting results
   */
  public double getJSON(CAMERA_POSITION position) {
    switch (position) {
      case LEFT_LOCALIZER:
        return m_leftLocalizer.getEntry("json").getDouble(0);
      case RIGHT_LOCALIZER:
        return m_rightLocalizer.getEntry("json").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Collects transformation/rotation data from limelight
   */
  public double[] getBotPose(CAMERA_POSITION position) {
    DriverStation.Alliance allianceColor = m_controls.getAllianceColor();
    double[] botPose;
    switch (position) {
      case LEFT_LOCALIZER:
        botPose = m_leftLocalizer.getEntry("botpose").getDoubleArray(defaultDoubleArray);

        if (botPose.length > 0) return botPose;
        //      case RIGHT_LOCALIZER:
        //        var rawBotPose = defaultDoubleArray;
        //        switch (allianceColor) {
        //          case Red:
        //            rawBotPose =
        //
        // m_rightLocalizer.getEntry("botpose_wpired").getDoubleArray(defaultDoubleArray);
        //            break;
        //          case Blue:
        //            rawBotPose =
        //
        // m_rightLocalizer.getEntry("botpose_wpiblue").getDoubleArray(defaultDoubleArray);
        //            break;
        //          default:
        //            rawBotPose =
        // m_rightLocalizer.getEntry("botpose").getDoubleArray(defaultDoubleArray);
        //
        //            if (rawBotPose.length > 0) {
        //              rawBotPose[0] = 15.980 / 2 + rawBotPose[0];
        //              rawBotPose[1] = 8.210 / 2 + rawBotPose[1];
        //              return rawBotPose;
        //            }
        //            break;
        //        }
      case RIGHT_LOCALIZER:
        botPose = m_rightLocalizer.getEntry("botpose").getDoubleArray(defaultDoubleArray);

        if (botPose.length > 0) return botPose;
      default:
        return defaultDoubleArray;
    }
    //    return defaultDoubleArray;
  }

  /**
   * Get the timestamp of the detection results.
   *
   * @return Robot Pose in meters
   */
  public double getDetectionTimestamp(CAMERA_POSITION position) {
    switch (position) {
      case LEFT_LOCALIZER:
        return m_leftLocalizer.getEntry("timestamp").getDouble(0);
      case RIGHT_LOCALIZER:
        return m_rightLocalizer.getEntry("timestamp").getDouble(0);
      default:
        return 0;
    }
  }

  public Pose2d getRobotPose2d(CAMERA_POSITION position) {
    double[] pose = getBotPose(position);
    return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
  }

  public Pose2d[] getRobotPoses2d(CAMERA_POSITION position) {
    Pose2d[] poseArray = {defaultPose};
    NetworkTable localizer = null;

    if (getValidTarget(position)) {
      switch (position) {
        case RIGHT_LOCALIZER:
          localizer = m_rightLocalizer;
          break;
        case LEFT_LOCALIZER:
          localizer = m_leftLocalizer;
          break;
      }
      robotPosX = localizer.getEntry("Robot Pose X").getDoubleArray(new double[] {});
      robotPosY = localizer.getEntry("Robot Pose Y").getDoubleArray(new double[] {});
      robotPosYaw = localizer.getEntry("Robot Pose Yaw").getDoubleArray(new double[] {});
      poseArray = new Pose2d[robotPosX.length];
      for (int i = 0; i < robotPosX.length; i++)
        poseArray[i] =
            new Pose2d(robotPosX[i], robotPosY[i], Rotation2d.fromDegrees(robotPosYaw[i]));
    }

    return poseArray;
  }

  public Pose2d[] getTagPoses2d(CAMERA_POSITION position) {
    NetworkTable localizer = null;
    Pose2d[] poseArray = {defaultPose};

    if (getValidTarget(position)) {
      switch (position) {
        case RIGHT_LOCALIZER:
          localizer = m_rightLocalizer;
          break;
        default:
        case LEFT_LOCALIZER:
          localizer = m_leftLocalizer;
          break;
      }
      try {
        tagPosX = localizer.getEntry("Tag Pose X").getDoubleArray(new double[] {});
        tagPosY = localizer.getEntry("Tag Pose Y").getDoubleArray(new double[] {});
        poseArray = new Pose2d[tagPosY.length];
        for (int i = 0; i < tagPosY.length; i++)
          poseArray[i] = new Pose2d(tagPosX[i], tagPosY[i], Rotation2d.fromDegrees(0));
      } catch (Exception e) {
        poseArray = new Pose2d[] {defaultPose};
      }
    }

    return poseArray;
  }

  public int[] getTagIds(CAMERA_POSITION position) {
    var tags = tagIds;
    if (getValidTarget(position)) {
      switch (position) {
        case LEFT_LOCALIZER:
          double[] rawTags = m_leftLocalizer.getEntry("tid").getDoubleArray(new double[] {});
          tags = DoubleStream.of(rawTags).mapToInt(d -> (int) d).toArray();
          break;
        case RIGHT_LOCALIZER:
          break;
      }
    }
    return tags;
  }

  private void updateVisionPose(CAMERA_POSITION position) {
    if (getValidTarget(position))
      m_swerveDrive
          .getOdometry()
          .addVisionMeasurement(getRobotPose2d(position), getDetectionTimestamp(position));
  }

  private void logData() {
    limelightTargetValid.append(getValidTargetType(CAMERA_POSITION.INTAKE));
    leftLocalizerTargetValid.append(getValidTargetType(CAMERA_POSITION.OUTTAKE));
  }

  @Override
  public void periodic() {
    var lCameraPos = Constants.Vision.cameraPositions[0];
    m_leftLocalizerPositionPub.set(new double[] {
            lCameraPos.getTranslation().getX(),
            lCameraPos.getTranslation().getY(),
            lCameraPos.getTranslation().getZ(),
            0,
            0,
//            -m_swerveDrive.getHeadingRotation2d().getRadians(),
            0,
            0,
    });
    var rCameraPos = Constants.Vision.cameraPositions[1];
    m_rightLocalizerPositionPub.set(new double[] {
            rCameraPos.getTranslation().getX(),
            rCameraPos.getTranslation().getY(),
            rCameraPos.getTranslation().getZ(),
            0,
            0,
//            -m_swerveDrive.getHeadingRotation2d().getRadians(),
            0,
            0
    });
    //    System.out.println("Vision Periodic");
    // This method will be called once per scheduler run
    updateVisionPose(CAMERA_POSITION.LEFT_LOCALIZER);
    //    updateVisionPose(CAMERA_POSITION.REAR_LOCALIZER);
    logData();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
