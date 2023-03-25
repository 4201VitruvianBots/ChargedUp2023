// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.CAN_UTIL_LIMIT;
import frc.robot.simulation.SimConstants;
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
  private DatagramSocket socket;
  private String receivedData = "";
  private double sensor1DistanceMeters;
  private double sensor2DistanceMeters;
  private double sensor3DistanceMeters;

  private CAN_UTIL_LIMIT limitCanUtil = CAN_UTIL_LIMIT.NORMAL;

  private Random rand = new Random();
  private Object obj;

  // Shuffleboard setup
  StringPublisher rawStringPub, gamePiecePub;
  DoublePublisher sensor1InchPub,
      sensor2InchPub,
      sensor1MMPub,
      sensor2MMPub,
      coneInchesPub,
      cubeInchesPub;
  
  // Mechanism2d visualization setup
  public Mechanism2d mech2d = new Mechanism2d(Constants.INTAKE.innerIntakeWidth * 1.5, Constants.INTAKE.innerIntakeWidth);
  public MechanismRoot2d leftBottomRoot = mech2d.getRoot("Intake Bottom Left", Constants.INTAKE.innerIntakeWidth * 0.25, Constants.INTAKE.innerIntakeWidth * 0.1);
  public MechanismRoot2d leftTopRoot = mech2d.getRoot("Intake Top Left", Constants.INTAKE.innerIntakeWidth * 0.25, Constants.INTAKE.innerIntakeWidth * 0.9);
  public MechanismRoot2d rightBottomRoot = mech2d.getRoot("Intake Bottom Right", Constants.INTAKE.innerIntakeWidth * 1.25, Constants.INTAKE.innerIntakeWidth * 0.1);
  public MechanismRoot2d coneRoot = mech2d.getRoot("Cone",
    Constants.INTAKE.innerIntakeWidth * 0.25 + Units.inchesToMeters(getConeDistanceInches()) - getConeWidthMeters() / 2,
    Constants.INTAKE.innerIntakeWidth * 0.1);
  public MechanismRoot2d cubeRoot = mech2d.getRoot("Cube",
    Constants.INTAKE.innerIntakeWidth * 0.25 + Units.inchesToMeters(getCubeDistanceInches()) - SimConstants.cubeWidth / 2,
    Constants.INTAKE.innerIntakeWidth * 0.9
  );
  public MechanismLigament2d leftIntakeLig =
      leftBottomRoot.append(new MechanismLigament2d("Intake Left", Constants.INTAKE.innerIntakeWidth * 0.8, 90));
  public MechanismLigament2d rightIntakeLig =
      rightBottomRoot.append(new MechanismLigament2d("Intake Right", Constants.INTAKE.innerIntakeWidth * 0.8, 90));
  public MechanismLigament2d coneIntakeLig =
      leftBottomRoot.append(new MechanismLigament2d("Cone Intake", Constants.INTAKE.innerIntakeWidth, 0));
  public MechanismLigament2d cubeIntakeLig =
      leftTopRoot.append(new MechanismLigament2d("Cube Intake", Constants.INTAKE.innerIntakeWidth, 0));
  public MechanismLigament2d coneLig = 
      coneRoot.append(new MechanismLigament2d("Cone", getConeWidthMeters(), 0));
  public MechanismLigament2d cubeLig = 
      cubeRoot.append(new MechanismLigament2d("Cone", SimConstants.cubeWidth, 0));

  /** Creates a new DistanceSensor. */
  public DistanceSensor() {
    if (RobotBase.isReal()) {
      try {
        InetAddress address = InetAddress.getByName("10.42.1.2"); // 239.42.01.1
        socket = new DatagramSocket(socketPort, address);
        socket.setReceiveBufferSize(512);
        socket.setSoTimeout(10);
      } catch (SocketException | UnknownHostException socketFail) {
        socketFail.printStackTrace();
      }
    }
    initSmartDashboard();
  }

  public String getRawSensorData() {
    return receivedData;
  }

  public double getSensorValueMillimeters(int sensor) {
    // This is a really stupid band-aid solution but I don't have time for anything else
    if (sensor == 0) sensor = 1;
    try {
      String sensorName = "sensor" + sensor + ".mm";

      // parsing string from recieved data
      obj = new JSONParser().parse(new StringReader(receivedData));

      // typecasting obj to JSONObject
      JSONObject jo = (JSONObject) obj;

      // getting sensor value
      long sensorValueLong = (long) jo.get(sensorName);

      // auto-unboxing does not go from Long to int directly, so
      double sensorValue = (double) sensorValueLong;

      return sensorValue;
    } catch (Exception e) {
      System.out.println("Boo hoo I can't read the file :_(");
      e.printStackTrace();
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
            + ",\"test\":"
            + rand.nextInt(100)
            + "}";
  }

  public Constants.INTAKE.HELD_GAMEPIECE getHeldGamepiece() {
    double leftConeSensorValue = getSensorValueMillimeters(Constants.INTAKE.leftConeSensorId) / 1000.0;
    double rightConeSensorValue = getSensorValueMillimeters(Constants.INTAKE.rightConeSensorId) / 1000.0;
    double cubeSensorValue = getSensorValueMillimeters(Constants.INTAKE.cubeSensorId) / 1000.0;

    if (leftConeSensorValue + rightConeSensorValue <= Constants.INTAKE.innerIntakeWidth) {
      return Constants.INTAKE.HELD_GAMEPIECE.CONE;
    }
    else if (cubeSensorValue <= Constants.INTAKE.innerIntakeWidth - 1) {
      return Constants.INTAKE.HELD_GAMEPIECE.CUBE;
    }
    else {
      return Constants.INTAKE.HELD_GAMEPIECE.NONE;
    }

  }

  // Returns the distance in inches from the left of the intake to the center of the game piece.
  // Works off 3 sensors, 2 for cone and 1 for cube
  public double getGamepieceDistanceInches(Constants.INTAKE.HELD_GAMEPIECE gamePiece) {
    double distanceMeters;

    double leftConeSensorValue = getSensorValueMillimeters(Constants.INTAKE.leftConeSensorId) / 1000.0;
    double rightConeSensorValue = getSensorValueMillimeters(Constants.INTAKE.rightConeSensorId) / 1000.0;
    double cubeSensorValue = getSensorValueMillimeters(Constants.INTAKE.cubeSensorId) / 1000.0;
    
    switch(gamePiece) {
      case CONE:
        // Reading cone sensors if cone in intake detected
        distanceMeters =
          leftConeSensorValue
            + ((Constants.INTAKE.innerIntakeWidth + leftConeSensorValue - rightConeSensorValue) / 2)
            - (Constants.INTAKE.innerIntakeWidth / 2);
        break;
      case CUBE:
        // Reading cube sensors if cube in intake detected
        distanceMeters =
            cubeSensorValue + (SimConstants.cubeWidth / 2) - (Constants.INTAKE.innerIntakeWidth / 2);
        break;
      default:
      case NONE:
        return 0;
    }

    // Clamp gamepiece distance
    distanceMeters = MathUtil.clamp(distanceMeters, 0, Constants.INTAKE.innerIntakeWidth);

    return Units.metersToInches(distanceMeters);
  }

  public double getGamepieceDistanceInches() {
    return getGamepieceDistanceInches(getHeldGamepiece());
  }

  public double getConeWidthMeters() {
    double leftConeSensorValue = getSensorValueMillimeters(Constants.INTAKE.leftConeSensorId) / 1000.0;
    double rightConeSensorValue = getSensorValueMillimeters(Constants.INTAKE.rightConeSensorId) / 1000.0;
    return (Constants.INTAKE.innerIntakeWidth + leftConeSensorValue - rightConeSensorValue) / 2;
  }

  public double getConeDistanceInches() {
    return getGamepieceDistanceInches(Constants.INTAKE.HELD_GAMEPIECE.CONE);
  }

  public double getCubeDistanceInches() {
    return getGamepieceDistanceInches(Constants.INTAKE.HELD_GAMEPIECE.CUBE);
  }

  public void setReduceCanUtilization(CAN_UTIL_LIMIT limitCan) {
    limitCanUtil = limitCan;
  }

  // Returns a pose where the center of the gamepiece should be
  public Pose2d getGamepiecePose(Pose2d intakePose) {
    return new Pose2d(
        intakePose.getX(),
        intakePose.getY() + getGamepieceDistanceInches(),
        intakePose.getRotation());
  }

  private void initSmartDashboard() {
    var distanceSensorTab =
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Distance Sensor");
    rawStringPub = distanceSensorTab.getStringTopic("Raw String Data").publish();
    gamePiecePub = distanceSensorTab.getStringTopic("Detected Gamepiece").publish();
    sensor1MMPub = distanceSensorTab.getDoubleTopic("Sensor 1 MM").publish();
    sensor2MMPub = distanceSensorTab.getDoubleTopic("Sensor 2 MM").publish();
    sensor1InchPub = distanceSensorTab.getDoubleTopic("Sensor 1 Inches").publish();
    sensor2InchPub = distanceSensorTab.getDoubleTopic("Sensor 2 Inches").publish();
    coneInchesPub = distanceSensorTab.getDoubleTopic("Cone Distance Inches").publish();
    cubeInchesPub = distanceSensorTab.getDoubleTopic("Cube Distance Inches").publish();

    coneIntakeLig.setColor(new Color8Bit(128, 0, 0));
    cubeIntakeLig.setColor(new Color8Bit(128, 0, 0));
    coneLig.setColor(new Color8Bit(255, 255, 0));
    cubeLig.setColor(new Color8Bit(128, 0, 128));
  }

  public void updateSmartDashboard(CAN_UTIL_LIMIT limitCan) {
    
    switch (limitCan) {
      case NORMAL:
        // Put not required stuff here
        SmartDashboard.putData("Intake Sim", mech2d);
        sensor1MMPub.set(getSensorValueMillimeters(1));
        sensor2MMPub.set(getSensorValueMillimeters(2));
        sensor1InchPub.set(getSensorValueInches(1));
        sensor2InchPub.set(getSensorValueInches(2));
        coneInchesPub.set(getConeDistanceInches());
        cubeInchesPub.set(getCubeDistanceInches());
        rawStringPub.set(receivedData);
        gamePiecePub.set(getHeldGamepiece().name());

        // Mech2d updates
        coneRoot.setPosition(Constants.INTAKE.innerIntakeWidth * 0.25 + Units.inchesToMeters(getConeDistanceInches()) - getConeWidthMeters() / 2,
          Constants.INTAKE.innerIntakeWidth * 0.1);
        cubeRoot.setPosition(Constants.INTAKE.innerIntakeWidth * 0.25 + Units.inchesToMeters(getCubeDistanceInches()) - SimConstants.cubeWidth / 2,
          Constants.INTAKE.innerIntakeWidth * 0.9);
        coneLig.setLength(getConeWidthMeters());
        coneIntakeLig.setLength(Units.inchesToMeters(getConeDistanceInches()));
        cubeIntakeLig.setLength(Units.inchesToMeters(getCubeDistanceInches()));
      default:
      case LIMITED:
        break;
    }
  }

  public void pollDistanceSensors() {
    // This method will be called once per scheduler run
    // testParserTab.setInteger(testParser());
    updateSmartDashboard(limitCanUtil);
    try {
      if (RobotBase.isSimulation()) {
        simulationPeriodic();
      } else {
        buffer = new byte[512];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        socket.receive(packet);

        receivedData = new String(packet.getData(), 0, packet.getLength());
      }
    } catch (SocketTimeoutException ex) {
      System.out.println("error: " + ex.getMessage());
      ex.printStackTrace();
    } catch (IOException ex) {
      System.out.println("Client error: " + ex.getMessage());
      ex.printStackTrace();
    } catch (Exception ex) {
      ex.printStackTrace();
    }
  }

  @Override
  public void close() throws Exception {
    if (socket != null) socket.close();
  }
}
