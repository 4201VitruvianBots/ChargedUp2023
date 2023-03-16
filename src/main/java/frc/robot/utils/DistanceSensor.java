// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.STATEHANDLER.INTAKING_STATES;
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

  private Random rand = new Random();
  private Object obj;

  // Shuffleboard setup
  StringPublisher rawStringPub;
  DoublePublisher sensor1InchPub,
      sensor2InchPub,
      sensor1MMPub,
      sensor2MMPub,
      coneInchesPub,
      cubeInchesPub;

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

  public INTAKE.HELD_GAMEPIECE getHeldGamepiece() {
    double leftConeSensorValue = getSensorValueMillimeters(Constants.INTAKE.leftConeSensorId) / 1000.0;
    double rightConeSensorValue = getSensorValueMillimeters(Constants.INTAKE.rightConeSensorId) / 1000.0;
    double cubeSensorValue = getSensorValueMillimeters(Constants.INTAKE.cubeSensorId) / 1000.0;

    if (leftConeSensorValue + rightConeSensorValue <= Constants.INTAKE.innerIntakeWidth) {
      return INTAKE.HELD_GAMEPIECE.CONE;
    }
    else if (cubeSensorValue <= Constants.INTAKE.innerIntakeWidth - 1) {
      return INTAKE.HELD_GAMEPIECE.CUBE;
    }
    else {
      return INTAKE.HELD_GAMEPIECE.NONE;
    }

  }

  // Returns the distance in inches from the left of the intake to the center of the game
  // piece. Negative if to the left, positive if to the right
  // Works off 3 sensors, 2 for cone and 1 for cube
  public double getGamepieceDistanceInches(INTAKE.HELD_GAMEPIECE gamePiece) {
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

    return Units.metersToInches(distanceMeters);
  }

  public double getGamepieceDistanceInches() {
    return getGamepieceDistanceInches(getHeldGamepiece());
  }

  public double getConeDistanceInches() {
    return getGamepieceDistanceInches(INTAKE.HELD_GAMEPIECE.CONE);
  }

  public double getCubeDistanceInches() {
    return getGamepieceDistanceInches(INTAKE.HELD_GAMEPIECE.CUBE);
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
        NetworkTableInstance.getDefault().getTable("Suffleboard").getSubTable("Distance Sensor");
    rawStringPub = distanceSensorTab.getStringTopic("Raw String Data").publish();
    sensor1MMPub = distanceSensorTab.getDoubleTopic("Sensor 1 MM").publish();
    sensor2MMPub = distanceSensorTab.getDoubleTopic("Sensor 2 MM").publish();
    sensor1InchPub = distanceSensorTab.getDoubleTopic("Sensor 1 Inches").publish();
    sensor2InchPub = distanceSensorTab.getDoubleTopic("Sensor 2 Inches").publish();
    coneInchesPub = distanceSensorTab.getDoubleTopic("Cone Distance Inches").publish();
    cubeInchesPub = distanceSensorTab.getDoubleTopic("Cube Distance Inches").publish();
  }

  public void updateSmartDashboard() {
    sensor1MMPub.set(getSensorValueMillimeters(1));
    sensor2MMPub.set(getSensorValueMillimeters(2));
    sensor1InchPub.set(getSensorValueInches(1));
    sensor2InchPub.set(getSensorValueInches(2));
    coneInchesPub.set(getConeDistanceInches());
    cubeInchesPub.set(getCubeDistanceInches());
    rawStringPub.set(receivedData);
  }

  public void pollDistanceSensors() {
    // This method will be called once per scheduler run
    // testParserTab.setInteger(testParser());
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
