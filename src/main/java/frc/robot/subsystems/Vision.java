// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.subsystems.StateHandler.m_chassisRoot2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.VISION;
import frc.robot.Constants.VISION.CAMERA_SERVER;
import frc.robot.Constants.VISION.PIPELINE;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;

import java.util.stream.DoubleStream;

public class Vision extends SubsystemBase implements AutoCloseable {
  private final SwerveDrive m_swerveDrive;
  private final Controls m_controls;
  private final Intake m_intakeSub;

  // Mech2d setup
  private MechanismLigament2d m_limelightLigament2d;

  private final NetworkTable m_intakeNt;
  private final NetworkTable m_leftLocalizer;
  private final NetworkTable m_rightLocalizer;
  private final NetworkTable m_fLocalizer;

  private final DoubleArrayPublisher m_leftLocalizerPositionPub;
  private final DoubleArrayPublisher m_rightLocalizerPositionPub;

  private final DoubleLogEntry limelightTargetValid;
  private final DoubleLogEntry leftLocalizerTargetValid;

  private final Timer searchPipelineTimer = new Timer();
  private final double searchPipelineWindow = 0.4;

  private final Timer searchTimer = new Timer();

  private double startTime, timestamp;
  private boolean timerStart;

  private INTAKE_STATE limelightState = INTAKE_STATE.NONE;
  private double m_pipeline;

  private final Pose2d defaultPose = new Pose2d(-5, -5, new Rotation2d());

  private final double[] defaultDoubleArray = {0, 0, 0, 0, 0, 0, 0};

  private final int[] tagIds = new int[10];
  double[] robotPosX = new double[10];
  double[] robotPosY = new double[10];
  double[] robotPosYaw = new double[10];

  double[] tagPosX = new double[10];
  double[] tagPosY = new double[10];

  public Vision(SwerveDrive swerveDrive, DataLog logger, Controls controls, Intake intake) {
    m_swerveDrive = swerveDrive;
    m_controls = controls;
    m_intakeSub = intake;

    m_intakeNt = NetworkTableInstance.getDefault().getTable("limelight");
    m_leftLocalizer = NetworkTableInstance.getDefault().getTable("lLocalizer");
    m_rightLocalizer = NetworkTableInstance.getDefault().getTable("rLocalizer");
    m_fLocalizer = NetworkTableInstance.getDefault().getTable("fusedLocalizer");

    PortForwarder.add(5800, CAMERA_SERVER.INTAKE.toString(), 5800);
    PortForwarder.add(5801, CAMERA_SERVER.INTAKE.toString(), 5801);
    PortForwarder.add(5802, CAMERA_SERVER.INTAKE.toString(), 5802);
    PortForwarder.add(5803, CAMERA_SERVER.INTAKE.toString(), 5803);
    PortForwarder.add(5804, CAMERA_SERVER.INTAKE.toString(), 5804);
    PortForwarder.add(5805, CAMERA_SERVER.INTAKE.toString(), 5805);
    PortForwarder.add(5806, CAMERA_SERVER.LEFT_LOCALIZER.toString(), 5800);
    PortForwarder.add(5807, CAMERA_SERVER.LEFT_LOCALIZER.toString(), 5801);
    PortForwarder.add(5808, CAMERA_SERVER.RIGHT_LOCALIZER.toString(), 5800);
    PortForwarder.add(5809, CAMERA_SERVER.RIGHT_LOCALIZER.toString(), 5801);

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
    resetPipelineSearch();
    initSmartDashboard();

    try {
      m_limelightLigament2d =
          m_chassisRoot2d.append(new MechanismLigament2d("Limelight", Units.inchesToMeters(8), 90));
      m_limelightLigament2d.setColor(new Color8Bit(0, 180, 40)); // Green
    } catch (Exception ignored) {

    }
  }

  public MechanismLigament2d getLimelightLigament() {
    return m_limelightLigament2d;
  }

  /**
   * Given a camera, return a boolean value based on if it sees a target or not.
   *
   * @return true: Camera has a target. false: Camera does not have a target
   */
  public boolean getValidTarget(CAMERA_SERVER location) {
    return getValidTargetType(location) > 0;
  }

  /*
   * Whether the limelight has any valid targets (0 or 1)
   */
  public double getValidTargetType(CAMERA_SERVER location) {
    switch (location) {
      case INTAKE:
        return m_intakeNt.getEntry("tv").getDouble(0);
      case LEFT_LOCALIZER:
        return m_leftLocalizer.getEntry("tv").getDouble(0);
      case RIGHT_LOCALIZER:
        return m_rightLocalizer.getEntry("tv").getDouble(0);
      case FUSED_LOCALIZER:
        return m_fLocalizer.getEntry("tv").getDouble(0);
      default:
        return 0;
    }
  }

  public double[] getAprilTagIds(CAMERA_SERVER location) {
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
  public double getTargetXAngle(CAMERA_SERVER location) {
    switch (location) {
      case INTAKE:
        return -m_intakeNt.getEntry("tx").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Vertical Offset From Crosshair To Target
   */
  public double getTargetYAngle(CAMERA_SERVER location) {
    switch (location) {
      case INTAKE:
        return m_intakeNt.getEntry("ty").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * The pipeline's latency contribution (ms). Add to "cl" to get total latency.
   */
  public double getCameraLatency(CAMERA_SERVER location) {
    switch (location) {
      case INTAKE:
        return m_intakeNt.getEntry("tl").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Target Area (0% of image to 100% of image)
   */
  public double getTargetArea(CAMERA_SERVER location) {
    switch (location) {
      case INTAKE:
        return m_intakeNt.getEntry("ta").getDouble(0);
      default:
        return 0;
    }
  }

  /*
   * Full JSON dump of targeting results
   */
  public double getJSON(CAMERA_SERVER location) {
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
  public void setPipeline(CAMERA_SERVER location, double pipeline) {
    m_pipeline = pipeline;
  }

  public void updatePipeline() {
    m_intakeNt.getEntry("pipeline").setDouble(1);
  }

  public double getPipeline(CAMERA_SERVER location) {
    switch (location) {
      case INTAKE:
        return m_intakeNt.getEntry("pipeline").getDouble(0);
      default:
        return 0.0;
    }
  }

  // false == cube, true == cone
  public boolean getGamePieceType(CAMERA_SERVER location) {
    switch (location) {
      case INTAKE:
        if (getPipeline(location) == 2.0 && searchLimelightTarget(location)) {
          return true; // cone
        } else if (getPipeline(location) == 1.0 && searchLimelightTarget(location)) {
          return false; // cube
        }
      default:
        return false;
    }
  }

  /*
   * resets timer for pipeline reconnection
   */
  public void resetSearch() {
    searchTimer.reset();
    searchTimer.start();
  }

  /*
   * resets timer for pipeline finder
   */
  public void resetPipelineSearch() {
    limelightState = INTAKE_STATE.NONE;
    searchPipelineTimer.reset();
    searchPipelineTimer.start();
  }

  /*
   * Look for any target
   */
  public boolean searchLimelightTarget(CAMERA_SERVER location) {
    //    if (getPipeline(location) == 1.0
    //        && m_intakeSub.getIntakeCubeState()) { // CUBE and if we're looking for cube
    //      return getValidTargetType(location) == 1.0
    //          && getTargetArea(location) > 1.0; // target read within threshold
    //    } else if (getPipeline(location) == 2.0
    //        && m_intakeSub.getIntakeConeState()) { // CONE and if we're looking for cone
    //      return getValidTargetType(location) == 1.0
    //          && getTargetArea(location) > 1.0; // target read within threshold
    //    }
    return false;
  }

  /*
   * Attempts to reconnect with pipeline within a timespan once target is lost
   */
  public void reconnectLimelightPipeline(CAMERA_SERVER location) {
    resetSearch();
    startTime = searchTimer.get();

    if (!timerStart && !searchLimelightTarget(location)) {
      timerStart = true;
      timestamp = searchTimer.get();
    } else if (timerStart && searchLimelightTarget(location)) {
      timestamp = 0;
      timerStart = false;
    }

    if (timestamp != 0 || searchTimer.get() - startTime > 3) {
      if (timerStart && searchTimer.get() - timestamp > 0.1 || searchTimer.get() - startTime > 2) {
        limelightState = INTAKE_STATE.NONE;
        searchLimelightPipeline(location);
      }
    }
  }

  /*
   * Look for a pipeline until a clear target is found when intaking
   */
  public void searchLimelightPipeline(CAMERA_SERVER location) {
    // Search
    if (limelightState == INTAKE_STATE.INTAKING_CONE
        || limelightState == INTAKE_STATE.INTAKING_CUBE) {
      int pipeline = (int) (Math.floor(searchPipelineTimer.get() / searchPipelineWindow) % 2) + 1;

      // threshold to find game object
      setPipeline(location, pipeline);
      // Try to find cube
      if (pipeline == PIPELINE.CUBE.get()) {
        if (getTargetArea(location) > 3.0) limelightState = INTAKE_STATE.INTAKING_CUBE;
      }
      if (pipeline == PIPELINE.CONE.get()) {
        if (getTargetArea(location) > 3.0) limelightState = INTAKE_STATE.INTAKING_CONE;
      }
    }

    // threshold to lose game object once it's found
    if (limelightState == INTAKE_STATE.INTAKING_CONE) {
      if (getTargetArea(location) < 2.0) {
        reconnectLimelightPipeline(location);
        limelightState = INTAKE_STATE.NONE;
      }
    }
    if (limelightState == INTAKE_STATE.INTAKING_CUBE) {
      if (getTargetArea(location) < 2.0) {
        reconnectLimelightPipeline(location);
        limelightState = INTAKE_STATE.NONE;
      }
    }
  }

  /*
   * Collects transformation/rotation data from limelight
   */
  public double[] getBotPose(CAMERA_SERVER location) {
    DriverStation.Alliance allianceColor = Controls.getAllianceColor();
    double[] botPose = new double[0];
    switch (location) {
      case LEFT_LOCALIZER:
        botPose = m_leftLocalizer.getEntry("botpose").getDoubleArray(defaultDoubleArray);
        break;
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
        break;
      case FUSED_LOCALIZER:
        botPose = m_fLocalizer.getEntry("botpose").getDoubleArray(defaultDoubleArray);
        break;
    }
    if (botPose.length > 0) return botPose;
    else return defaultDoubleArray;
  }

  /**
   * Get the timestamp of the detection results.
   *
   * @return Robot Pose in meters
   */
  public double getDetectionTimestamp(CAMERA_SERVER location) {
    switch (location) {
      case LEFT_LOCALIZER:
        return m_leftLocalizer.getEntry("timestamp").getDouble(0);
      case RIGHT_LOCALIZER:
        return m_rightLocalizer.getEntry("timestamp").getDouble(0);
      case FUSED_LOCALIZER:
        return m_fLocalizer.getEntry("timestamp").getDouble(0);
      default:
        return 0;
    }
  }

  public Pose2d getRobotPose2d(CAMERA_SERVER location) {
    double[] pose = getBotPose(location);
    return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
  }

  public Pose2d[] getRobotPoses2d(CAMERA_SERVER location) {
    Pose2d[] poseArray = {defaultPose};
    NetworkTable localizer = null;

    if (getValidTarget(location)) {
      switch (location) {
        case INTAKE:
          break;
        case RIGHT_LOCALIZER:
          localizer = m_rightLocalizer;
          break;
        case LEFT_LOCALIZER:
          localizer = m_leftLocalizer;
          break;
        case FUSED_LOCALIZER:
          localizer = m_fLocalizer;
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

  public Pose2d[] getTagPoses2d(CAMERA_SERVER location) {
    NetworkTable localizer;
    Pose2d[] poseArray = {defaultPose};

    if (getValidTarget(location)) {
      switch (location) {
        case LEFT_LOCALIZER:
          localizer = m_leftLocalizer;
          break;
        case RIGHT_LOCALIZER:
          localizer = m_rightLocalizer;
          break;
        default:
        case FUSED_LOCALIZER:
          localizer = m_fLocalizer;
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

  public int[] getTagIds(CAMERA_SERVER location) {
    var tags = tagIds;
    double[] rawTags;
    if (getValidTarget(location)) {
      switch (location) {
        case LEFT_LOCALIZER:
          rawTags = m_leftLocalizer.getEntry("tid").getDoubleArray(new double[] {});
          tags = DoubleStream.of(rawTags).mapToInt(d -> (int) d).toArray();
          break;
        case RIGHT_LOCALIZER:
          rawTags = m_rightLocalizer.getEntry("tid").getDoubleArray(new double[] {});
          tags = DoubleStream.of(rawTags).mapToInt(d -> (int) d).toArray();
          break;
        case FUSED_LOCALIZER:
          rawTags = m_fLocalizer.getEntry("tid").getDoubleArray(new double[] {});
          tags = DoubleStream.of(rawTags).mapToInt(d -> (int) d).toArray();
          break;
        default:
        case INTAKE:
          break;
      }
    }
    return tags;
  }

  private void updateVisionPose(CAMERA_SERVER location) {
    if (getValidTarget(location))
      m_swerveDrive
          .getOdometry()
          .addVisionMeasurement(getRobotPose2d(location), getDetectionTimestamp(location));
  }

  private void logData() {
    limelightTargetValid.append(getValidTargetType(CAMERA_SERVER.INTAKE));
  }

  public void initSmartDashboard() {
    SmartDashboard.putData(this);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("pipeline", getPipeline(CAMERA_SERVER.INTAKE));
  }

  @Override
  public void periodic() {
    m_leftLocalizerPositionPub.set(
        new double[] {
          VISION.LOCALIZER_CAMERA_POSITION[0].getTranslation().getX(),
          VISION.LOCALIZER_CAMERA_POSITION[0].getTranslation().getY(),
          VISION.LOCALIZER_CAMERA_POSITION[0].getTranslation().getZ(),
          0,
          0,
          0
        });
    m_rightLocalizerPositionPub.set(
        new double[] {
          VISION.LOCALIZER_CAMERA_POSITION[1].getTranslation().getX(),
          VISION.LOCALIZER_CAMERA_POSITION[1].getTranslation().getY(),
          VISION.LOCALIZER_CAMERA_POSITION[1].getTranslation().getZ(),
          0,
          0,
          0
        });
    // This method will be called once per scheduler run
    updateSmartDashboard();
    // updateVisionPose(CAMERA_SERVER.FUSED_LOCALIZER);
    // searchLimelightPipeline(CAMERA_SERVER.INTAKE);
    updatePipeline();
    // searchforCube(CAMERA_SERVER.INTAKE, 1.0);
    logData();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    if (m_limelightLigament2d != null) m_limelightLigament2d.close();
  }
}
