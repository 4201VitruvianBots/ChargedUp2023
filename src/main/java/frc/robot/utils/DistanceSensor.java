// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.INTAKE.INTAKE_STATE;
import frc.robot.Constants.INTAKE.SENSOR_STATUS;
import frc.robot.Constants.STATE_HANDLER;
import frc.robot.simulation.SimConstants;
import frc.robot.utils.logging.AdvantageDoublePublisher;
import frc.robot.utils.logging.AdvantageStringPublisher;
import frc.robot.utils.logging.LoggingUtils;

import java.io.IOException;
import java.io.StringReader;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;
import java.util.Random;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class DistanceSensor implements AutoCloseable {
  private final int socketPort = 25000;

  private byte[] buffer = new byte[512];
  private final double[] sensorValuesMM = new double[] {0, 0, 0};

  private DatagramSocket socket;
  private String receivedData = "";
  private boolean isInitialized = false;

  private final boolean m_limitCanUtil = STATE_HANDLER.limitCanUtilization;

  private final Random rand = new Random();
  private Object obj;

  // Shuffleboard setup
  AdvantageStringPublisher rawStringPub;
  AdvantageDoublePublisher sensor1InchPub,
      sensor2InchPub,
      sensor3InchPub,
      sensor1MMPub,
      sensor2MMPub,
      sensor3MMPub,
      coneInchesPub,
      cubeInchesPub;

  // Mechanism2d visualization setup
  private final Mechanism2d mech2d =
      new Mechanism2d(INTAKE.innerIntakeWidth * 1.5, INTAKE.innerIntakeWidth);
  private final MechanismRoot2d leftBottomRoot =
      mech2d.getRoot(
          "Intake Bottom Left", INTAKE.innerIntakeWidth * 0.25, INTAKE.innerIntakeWidth * 0.1);
  private final MechanismRoot2d leftTopRoot =
      mech2d.getRoot(
          "Intake Top Left", INTAKE.innerIntakeWidth * 0.25, INTAKE.innerIntakeWidth * 0.9);
  private final MechanismRoot2d rightBottomRoot =
      mech2d.getRoot(
          "Intake Bottom Right", INTAKE.innerIntakeWidth * 1.25, INTAKE.innerIntakeWidth * 0.1);
  private final MechanismRoot2d coneRoot =
      mech2d.getRoot(
          "Cone",
          INTAKE.innerIntakeWidth * 0.25
              + Units.inchesToMeters(getConeDistanceInches())
              - getConeWidthMeters() / 2,
          INTAKE.innerIntakeWidth * 0.1);
  private final MechanismRoot2d cubeRoot =
      mech2d.getRoot(
          "Cube",
          INTAKE.innerIntakeWidth * 0.25
              + Units.inchesToMeters(getCubeDistanceInches())
              - SimConstants.cubeWidth / 2,
          INTAKE.innerIntakeWidth * 0.9);
  private final MechanismLigament2d leftIntakeLig =
      leftBottomRoot.append(
          new MechanismLigament2d("IntakeLeft", INTAKE.innerIntakeWidth * 0.8, 90));
  private final MechanismLigament2d rightIntakeLig =
      rightBottomRoot.append(
          new MechanismLigament2d("IntakeRight", INTAKE.innerIntakeWidth * 0.8, 90));
  private final MechanismLigament2d coneIntakeLig =
      leftBottomRoot.append(new MechanismLigament2d("ConeIntake", INTAKE.innerIntakeWidth, 0));
  private final MechanismLigament2d cubeIntakeLig =
      leftTopRoot.append(new MechanismLigament2d("CubeIntake", INTAKE.innerIntakeWidth, 0));
  private final MechanismLigament2d coneLig =
      coneRoot.append(new MechanismLigament2d("Cone", getConeWidthMeters(), 0));
  private final MechanismLigament2d cubeLig =
      cubeRoot.append(new MechanismLigament2d("Cone", SimConstants.cubeWidth, 0));

  /** Creates a new DistanceSensor. */
  public DistanceSensor() {
    try {
      InetAddress address = InetAddress.getByName("10.42.1.2"); // 239.42.01.1
      socket = new DatagramSocket(socketPort, address);
      socket.setReceiveBufferSize(512);
      socket.setSoTimeout(10);
      isInitialized = true;
    } catch (SocketException | UnknownHostException socketFail) {
      //        socketFail.printStackTrace();
    }
    initSmartDashboard();
  }

  public String getRawSensorData() {
    return receivedData;
  }

  public double getSensorValueMillimeters(int sensor) {
    try {
      String sensorName = "sensor" + sensor + ".mm";

      // parsing string from received data
      obj = new JSONParser().parse(new StringReader(receivedData));

      // typecasting obj to JSONObject
      JSONObject jo = (JSONObject) obj;

      // getting sensor value
      long sensorValueLong = (long) jo.get(sensorName);

      // auto-unboxing does not go from Long to int directly, so
      double sensorValue = (double) sensorValueLong;

      return sensorValue;
    } catch (Exception e) {
      // System.out.println("Failed to get sensor " + Integer.toString(sensor) + " value");
      //      e.printStackTrace();
      return -1;
    }
  }

  public double getSensorValueInches(int sensor) {
    return Units.metersToInches(getSensorValueMillimeters(sensor) / 1000);
  }

  public void simulationPeriodic() {
    receivedData =
        "{\"sensor1.mm\":"
            + rand.nextInt(394)
            + ",\"sensor2.mm\":"
            + rand.nextInt(394)
            + ",\"sensor3.mm\":"
            + rand.nextInt(394)
            + ",\"test\":"
            + rand.nextInt(100)
            + ",\"sensor1.status\":"
            + "connected"
            + ",\"sensor2.status\":"
            + "connected"
            + ",\"sensor3.status\":"
            + "connected"
            + "}";
  }

  // Returns the distance in inches from the left of the intake to the center of the game piece.
  // Works off 3 sensors, 2 for cone and 1 for cube
  public double getGamepieceDistanceInches(INTAKE.INTAKE_STATE gamePiece) {
    double distanceMeters;

    double leftConeSensorValue = getSensorValueMillimeters(INTAKE.leftConeSensorId) / 1000.0;
    double rightConeSensorValue = getSensorValueMillimeters(INTAKE.rightConeSensorId) / 1000.0;
    double cubeSensorValue = getSensorValueMillimeters(INTAKE.cubeSensorId) / 1000.0;

    switch (gamePiece) {
      case HOLDING_CONE:
        // Reading cone sensors if cone in intake detected
        distanceMeters =
            leftConeSensorValue
                + ((INTAKE.innerIntakeWidth + leftConeSensorValue - rightConeSensorValue) / 2)
                - (INTAKE.innerIntakeWidth / 2);
        break;
      case HOLDING_CUBE:
        // Reading cube sensors if cube in intake detected
        distanceMeters =
            cubeSensorValue + (SimConstants.cubeWidth / 2) - (INTAKE.innerIntakeWidth / 2);
        break;
      default:
      case NONE:
        return 0;
    }

    // Clamp gamepiece distance
    distanceMeters = MathUtil.clamp(distanceMeters, 0, INTAKE.innerIntakeWidth);

    return Units.metersToInches(distanceMeters);
  }

  public double getConeWidthMeters() {
    double leftConeSensorValue = getSensorValueMillimeters(INTAKE.leftConeSensorId) / 1000.0;
    double rightConeSensorValue = getSensorValueMillimeters(INTAKE.rightConeSensorId) / 1000.0;
    return (INTAKE.innerIntakeWidth + leftConeSensorValue - rightConeSensorValue) / 2;
  }

  public double getConeDistanceInches() {
    return getGamepieceDistanceInches(INTAKE_STATE.INTAKING_CONE);
  }

  public double getCubeDistanceInches() {
    return getGamepieceDistanceInches(INTAKE.INTAKE_STATE.HOLDING_CONE);
  }

  private void initSmartDashboard() {
    var distanceSensorTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("DistanceSensor");
    rawStringPub.publish(distanceSensorTab, "Raw String Data");
    sensor1MMPub.publish(distanceSensorTab, "Sensor1MM");
    sensor2MMPub.publish(distanceSensorTab, "Sensor2MM");
    sensor3MMPub.publish(distanceSensorTab, "Sensor3MM");
    sensor1InchPub.publish(distanceSensorTab, "Sensor1Inches");
    sensor2InchPub.publish(distanceSensorTab, "Sensor2Inches");
    sensor3InchPub.publish(distanceSensorTab, "Sensor3Inches");
    coneInchesPub.publish(distanceSensorTab, "ConeDistanceInches");
    cubeInchesPub.publish(distanceSensorTab, "CubeDistanceInches");

    coneIntakeLig.setColor(new Color8Bit(128, 0, 0));
    cubeIntakeLig.setColor(new Color8Bit(128, 0, 0));
    coneLig.setColor(new Color8Bit(255, 255, 0));
    cubeLig.setColor(new Color8Bit(128, 0, 128));

    SmartDashboard.putData("Intake Sim", mech2d);
  }

  public boolean isInitialized() {
    return isInitialized;
  }

  public INTAKE.SENSOR_STATUS getSensorStatus(int sensor) {
    try {
      String sensorName = "sensor" + sensor + ".status";

      // parsing string from received data
      obj = new JSONParser().parse(new StringReader(receivedData));

      // typecasting obj to JSONObject
      JSONObject jo = (JSONObject) obj;

      // getting sensor status
      String status = (String) jo.get(sensorName);

      switch (status) {
        case "failed":
          return SENSOR_STATUS.FAILED;
        case "disconnected":
          return SENSOR_STATUS.DISCONNECTED;
        case "timeout":
          return SENSOR_STATUS.TIMEOUT;
        case "connected":
          return SENSOR_STATUS.CONNECTED;
        default:
          return SENSOR_STATUS.UNREPORTED;
      }

    } catch (Exception e) {
      // System.out.println("Boo hoo I can't read the file :_(");
      e.printStackTrace();
      return SENSOR_STATUS.UNREPORTED;
    }
  }

  public void updateSmartDashboard() {
    if (!m_limitCanUtil) {
      // Put not required stuff here
      sensor1MMPub.set(getSensorValueMillimeters(1));
      sensor2MMPub.set(getSensorValueMillimeters(2));
      sensor3MMPub.set(getSensorValueMillimeters(3));
      sensor1InchPub.set(getSensorValueInches(1));
      sensor2InchPub.set(getSensorValueInches(2));
      sensor3InchPub.set(getSensorValueInches(3));
      coneInchesPub.set(getConeDistanceInches());
      cubeInchesPub.set(getCubeDistanceInches());
      rawStringPub.set(receivedData);

      // Mech2d updates
      //      coneRoot.setPosition(
      //          INTAKE.innerIntakeWidth * 0.25
      //              + Units.inchesToMeters(getConeDistanceInches())
      //              - getConeWidthMeters() / 2,
      //          INTAKE.innerIntakeWidth * 0.1);
      //      cubeRoot.setPosition(
      //          INTAKE.innerIntakeWidth * 0.25
      //              + Units.inchesToMeters(getCubeDistanceInches())
      //              - SimConstants.cubeWidth / 2,
      //          INTAKE.innerIntakeWidth * 0.9);
      //      coneLig.setLength(getConeWidthMeters());
      //      coneIntakeLig.setLength(Units.inchesToMeters(getConeDistanceInches()));
      //      cubeIntakeLig.setLength(Units.inchesToMeters(getCubeDistanceInches()));
    }
  }

  public void pollDistanceSensors() {
    // This method will be called once per scheduler run
    // testParserTab.setInteger(testParser());
    try {
      //      if (RobotBase.isSimulation()) {
      //        simulationPeriodic();
      //      } else {
      buffer = new byte[512];
      DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
      socket.receive(packet);

      receivedData = new String(packet.getData(), 0, packet.getLength());

      for (int i = 1; i <= 3; i++) {
        String sensorName = "sensor" + i + ".mm";

        // parsing string from received data
        obj = new JSONParser().parse(new StringReader(receivedData));

        // typecasting obj to JSONObject
        JSONObject jo = (JSONObject) obj;

        // getting sensor value
        long sensorValueLong = (long) jo.get(sensorName);

        // auto-unboxing does not go from Long to int directly, so
        sensorValuesMM[i - 1] = (double) sensorValueLong;
      }

      //      }
    } catch (SocketTimeoutException ex) {
      //      System.out.println("DistanceSensor-SocketTimeoutError");
      //      ex.printStackTrace();
    } catch (IOException ex) {
      //      System.out.println("DistanceSensor-IOError");
      //      ex.printStackTrace();
    } catch (Exception ex) {
      //      System.out.println("DistanceSensor-UnknownError");
      //      ex.printStackTrace();
    }
  }

  public void periodic() {
    updateSmartDashboard();
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    if (socket != null) socket.close();
  }
}
