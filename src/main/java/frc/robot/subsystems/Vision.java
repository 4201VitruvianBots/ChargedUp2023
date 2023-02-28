// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Vision.CAMERA_LOCATION;
import java.util.stream.DoubleStream;

public class Vision extends SubsystemBase {

  private final SwerveDrive m_swerveDrive;
  private final Controls m_controls;
  private final Intake m_intakeSub;

  private final NetworkTable m_intakeNet;
  private final NetworkTable outtake;
  private final NetworkTable m_leftLocalizer;
  private final NetworkTable m_rightLocalizer;

  private final DoubleArrayPublisher m_leftLocalizerPositionPub;
  private final DoubleArrayPublisher m_rightLocalizerPositionPub;

  private DoubleLogEntry limelightTargetValid;
  private DoubleLogEntry leftLocalizerTargetValid;

  private Timer searchTimer = new Timer();
  private Double searchWindow = 0.2;

  private double desiredAngleSetpoint;

  private DoublePublisher kSetpointPub;

  private enum targetType {
    INTAKING,
    CONE,
    CUBE,
    NONE
  }

  private targetType targetFound = targetType.NONE;

  Pose2d defaultPose = new Pose2d(-5, -5, new Rotation2d());

  double[] defaultDoubleArray = {0, 0, 0, 0, 0, 0};

  int[] tagIds = new int[10];
  double[] robotPosX = new double[10];
  double[] robotPosY = new double[10];
  double[] robotPosYaw = new double[10];

  double[] tagPosX = new double[10];
  double[] tagPosY = new double[10];

  public Vision(SwerveDrive swerveDrive, DataLog logger, Controls controls, Intake intake) {
    m_swerveDrive = swerveDrive;
    m_controls = controls;
    m_intakeSub = intake;

    m_intakeNet = NetworkTableInstance.getDefault().getTable("limelight");
    outtake = NetworkTableInstance.getDefault().getTable("limelight");
    m_leftLocalizer = NetworkTableInstance.getDefault().getTable("lLocalizer");
    m_rightLocalizer = NetworkTableInstance.getDefault().getTable("rLocalizer");

    PortForwarder.add(5800, Constants.Vision.SERVER_IPS.INTAKE.toString(), 5800);
    PortForwarder.add(5801, Constants.Vision.SERVER_IPS.INTAKE.toString(), 5801);
    PortForwarder.add(5802, Constants.Vision.SERVER_IPS.INTAKE.toString(), 5802);
    PortForwarder.add(5803, Constants.Vision.SERVER_IPS.INTAKE.toString(), 5803);
    PortForwarder.add(5804, Constants.Vision.SERVER_IPS.INTAKE.toString(), 5804);
    PortForwarder.add(5805, Constants.Vision.SERVER_IPS.INTAKE.toString(), 5805);
    PortForwarder.add(5806, Constants.Vision.SERVER_IPS.LEFT_LOCALIZER.toString(), 5800);
    PortForwarder.add(5807, Constants.Vision.SERVER_IPS.LEFT_LOCALIZER.toString(), 5801);
    PortForwarder.add(5808, Constants.Vision.SERVER_IPS.RIGHT_LOCALIZER.toString(), 5800);
    PortForwarder.add(5809, Constants.Vision.SERVER_IPS.RIGHT_LOCALIZER.toString(), 5801);

    limelightTargetValid = new DoubleLogEntry(logger, "/vision/limelight_tv");
    leftLocalizerTargetValid = new DoubleLogEntry(logger, "/vision/fLocalizer_tv");

    m_leftLocalizerPositionPub =
        NetworkTableInstance.getDefault()
            .getTable("lLocalizer")
            .getDoubleArrayTopic("camToRobotT3D")
            .publish();

    m_rightLocalizerPositionPub =
        NetworkTableInstance.getDefault()
            .getTable("rLocalizer")
            .getDoubleArrayTopic("camToRobotT3D")
            .publish();

    resetSearch();
  }

  /**
   * Given a camera, return a boolean value based on if it sees a target or not.
   *
   * @return true: Camera has a target. false: Camera does not have a target
   */
  public boolean getValidTarget(CAMERA_LOCATION location) {
    return getValidTargetType(location) > 0;
  }

  /*
   * Whether the limelight has any valid targets (0 or 1)
   */
  public double getValidTargetType(CAMERA_LOCATION location) {
    switch (location) {
      case INTAKE:
        return m_intakeNet.getEntry("tv").getDouble(0);
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

  public double[] getAprilTagIds(CAMERA_LOCATION location) {
    switch (location) {
      case LEFT_LOCALIZER:
        return m_leftLocalizer.getEntry("tid").getDoubleArray(defaultDoubleArray);
      case RIGHT_LOCALIZER:
        return m_rightLocalizer.getEntry("tid").getDoubleArray(defaultDoubleArray);
      default:
        return defaultDoubleArray;
    }
  }

  /*
   * Horizontal Offset From Crosshair To Target
   */
  public double getTargetXAngle(CAMERA_LOCATION location) {
    switch (location) {
      case INTAKE:
        return -m_intakeNet.getEntry("tx").getDouble(0);
      case OUTTAKE:
        return -outtake.getEntry("tx").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Vertical Offset From Crosshair To Target
   */
  public double getTargetYAngle(CAMERA_LOCATION location) {
    switch (location) {
      case INTAKE:
        return m_intakeNet.getEntry("ty").getDouble(0);
      case OUTTAKE:
        return outtake.getEntry("ty").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * The pipeline’s latency contribution (ms). Add to “cl” to get total latency.
   */
  public double getCameraLatency(CAMERA_LOCATION location) {
    switch (location) {
      case INTAKE:
        return m_intakeNet.getEntry("tl").getDouble(0);
      case OUTTAKE:
        return outtake.getEntry("tl").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Target Area (0% of image to 100% of image)
   */
  public double getTargetArea(CAMERA_LOCATION location) {
    switch (location) {
      case INTAKE:
        return m_intakeNet.getEntry("ta").getDouble(0);
      case OUTTAKE:
        return outtake.getEntry("ta").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Full JSON dump of targeting results
   */
  public double getJSON(CAMERA_LOCATION location) {
    switch (location) {
      case LEFT_LOCALIZER:
        return m_leftLocalizer.getEntry("json").getDouble(0);
      case RIGHT_LOCALIZER:
        return m_rightLocalizer.getEntry("json").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Pipeline 1 = cube
   * Pipeline 2 = cone
   */
  public void setPipeline(CAMERA_LOCATION location, double pipeline) {
    switch (location) {
      case INTAKE:
        m_intakeNet.getEntry("pipeline").setDouble(pipeline);
        break;
    }
  }

  public double getPipeline(CAMERA_LOCATION location) {
    switch (location) {
      case INTAKE:
        return m_intakeNet.getEntry("pipeline").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * resets timer for pipeline finder
   */
  public void resetSearch() {
    targetFound = targetType.NONE;
    searchTimer.reset();
    searchTimer.start();
  }

  /*
   * Look for any target
   */
  public boolean searchLimelightTarget(CAMERA_LOCATION location) {
    if (getPipeline(location) == 1.0) { // CUBE
      return getValidTargetType(location) == 1.0
          && getTargetArea(location) > 3.0; // target read and threshold
    } else if (getPipeline(location) == 2.0) { // CONE
      return getValidTargetType(location) == 1.0
          && getTargetArea(location) > 3.0; // target read and threshold
    }
    return false;
  }

  /*
   * Look for a pipeline until a clear target is found when intaking
   */
  public void searchLimelightPipeline(CAMERA_LOCATION location) {
    if (m_intakeSub.getIntakeState()) {
      int pipeline = (int) (Math.floor(searchTimer.get() / searchWindow) % 2) + 1;

      // threshold to find game object
      if (targetFound == targetType.NONE || targetFound == targetType.INTAKING) {
        setPipeline(location, pipeline);
        if (getTargetArea(location) > 3.0 && pipeline == 1) {
          targetFound = targetType.CUBE;
        } else if (getTargetArea(location) > 3.0 && pipeline == 2) {
          targetFound = targetType.CONE;
        }
      }

      // threshold to lose game object once it's found
      if (targetFound == targetType.CUBE) {
        if (getTargetArea(location) < 2.0) {
          targetFound = targetType.NONE;
        }
      }
      if (targetFound == targetType.CONE) {
        if (getTargetArea(location) < 2.0) {
          targetFound = targetType.NONE;
        }
      }
    }
  }

  /*
   * Collects transformation/rotation data from limelight
   */
  public double[] getBotPose(CAMERA_LOCATION location) {
    DriverStation.Alliance allianceColor = m_controls.getAllianceColor();
    double[] botPose;
    switch (location) {
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
  public double getDetectionTimestamp(CAMERA_LOCATION location) {
    switch (location) {
      case LEFT_LOCALIZER:
        return m_leftLocalizer.getEntry("timestamp").getDouble(0);
      case RIGHT_LOCALIZER:
        return m_rightLocalizer.getEntry("timestamp").getDouble(0);
      default:
        return 0;
    }
  }

  public Pose2d getRobotPose2d(CAMERA_LOCATION location) {
    double[] pose = getBotPose(location);
    return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
  }

  public Pose2d[] getRobotPoses2d(CAMERA_LOCATION location) {
    Pose2d[] poseArray = {defaultPose};
    NetworkTable localizer = null;

    if (getValidTarget(location)) {
      switch (location) {
        case RIGHT_LOCALIZER:
          localizer = m_rightLocalizer;
          break;
        case LEFT_LOCALIZER:
          localizer = m_leftLocalizer;
          break;
        case INTAKE:
          break;
        case OUTTAKE:
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

  public Pose2d[] getTagPoses2d(CAMERA_LOCATION location) {
    NetworkTable localizer = null;
    Pose2d[] poseArray = {defaultPose};

    if (getValidTarget(location)) {
      switch (location) {
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

  public int[] getTagIds(CAMERA_LOCATION location) {
    var tags = tagIds;
    if (getValidTarget(location)) {
      switch (location) {
        case LEFT_LOCALIZER:
          double[] rawTags = m_leftLocalizer.getEntry("tid").getDoubleArray(new double[] {});
          tags = DoubleStream.of(rawTags).mapToInt(d -> (int) d).toArray();
          break;
        case RIGHT_LOCALIZER:
          break;
        case INTAKE:
          break;
        case OUTTAKE:
          break;
      }
    }
    return tags;
  }

  private void updateVisionPose(CAMERA_LOCATION location) {
    if (getValidTarget(location))
      m_swerveDrive
          .getOdometry()
          .addVisionMeasurement(getRobotPose2d(location), getDetectionTimestamp(location));
  }

  private void logData() {
    limelightTargetValid.append(getValidTargetType(CAMERA_LOCATION.INTAKE));
    leftLocalizerTargetValid.append(getValidTargetType(CAMERA_LOCATION.OUTTAKE));
  }

  @Override
  public void periodic() {
    m_leftLocalizerPositionPub.set(
        new double[] {
          Constants.Vision.LOCALIZER_CAMERA_POSITION[0].getTranslation().getX(),
          Constants.Vision.LOCALIZER_CAMERA_POSITION[0].getTranslation().getY(),
          Constants.Vision.LOCALIZER_CAMERA_POSITION[0].getTranslation().getZ(),
          0,
          0,
          0
        });
    m_rightLocalizerPositionPub.set(
        new double[] {
          Constants.Vision.LOCALIZER_CAMERA_POSITION[1].getTranslation().getX(),
          Constants.Vision.LOCALIZER_CAMERA_POSITION[1].getTranslation().getY(),
          Constants.Vision.LOCALIZER_CAMERA_POSITION[1].getTranslation().getZ(),
          0,
          0,
          0
        });
    //    System.out.println("Vision Periodic");
    // This method will be called once per scheduler run
    updateVisionPose(CAMERA_LOCATION.LEFT_LOCALIZER);
    //    updateVisionPose(CAMERA_LOCATION.REAR_LOCALIZER);
    searchLimelightPipeline(CAMERA_LOCATION.INTAKE);
    logData();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
