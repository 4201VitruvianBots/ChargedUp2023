// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.StateHandler.INTAKING_STATES;
import java.io.File;
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

public class DistanceSensor {
  private final int socketPort = 25000;

  private DatagramSocket socket;
  private String receivedData;

  private Random rand = new Random();

  // Shuffleboard setup

  public static ShuffleboardTab distanceSensorTab = Shuffleboard.getTab("DistanceSensor");

  public GenericEntry rawStringTab = distanceSensorTab.add("Raw String Data", "None").getEntry();
  public GenericEntry sensorValue1MMTab = distanceSensorTab.add("Sensor Value 1 Millimeters", 0).getEntry();
  public GenericEntry sensorValue2MMTab = distanceSensorTab.add("Sensor Value 2 Millimeters", 0).getEntry();
  public GenericEntry sensorValue1InTab = distanceSensorTab.add("Sensor Value 1 Inches", 0).getEntry();
  public GenericEntry sensorValue2InTab = distanceSensorTab.add("Sensor Value 2 Inches", 0).getEntry();
  public GenericEntry coneInchesTab = distanceSensorTab.add("Cone Distance Inches", 0).getEntry();
  public GenericEntry cubeInchesTab = distanceSensorTab.add("Cube Distance Inches", 0).getEntry();

  /** Creates a new DistanceSensor. */
  public DistanceSensor() {
    try {
      InetAddress address = InetAddress.getByName("10.42.1.2"); // 239.42.01.1
      socket = new DatagramSocket(socketPort, address);
      socket.setSoTimeout(1000);
    } catch (SocketException | UnknownHostException socketFail) {
      socketFail.printStackTrace();
    }
  }

  public String getRawSensorData() {
    return receivedData;
  }

  public double getSensorValueMillimeters(int sensor) {
    // This is a really stupid band-aid solution but I don't have time for anything else
    if (sensor == 0)
      sensor = 1;
    try {
      String sensorName = "sensor" + Integer.toString(sensor) + ".mm";

      // parsing string from recieved data
      Object obj = new JSONParser().parse(new StringReader(receivedData));

      // typecasting obj to JSONObject
      JSONObject jo = (JSONObject) obj;

      // getting sensor value
      long sensorValueLong = (long) jo.get(sensorName);

      // auto-unboxing does not go from Long to int directly, so
      double sensorValue = (double) (long) sensorValueLong;

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
            + Integer.toString(rand.nextInt(394))
            + ",\"sensor2.mm\":"
            + Integer.toString(rand.nextInt(394))
            + ",\"test\":"
            + Integer.toString(rand.nextInt(100))
            + "}";
  }

  // Returns the distance in inches from the left of the intake to the center of the game
  // piece. Negative if to the left, positive if to the right
  // Works off 3 sensors, 2 for cone and 1 for cube
  public double getGamepieceDistanceInches(INTAKING_STATES intakeState) {
    double distanceMeters;

    switch (intakeState) {
      case CONE:
        int leftSensorId = Constants.INTAKE.leftConeSensorId;
        int rightSensorId = Constants.INTAKE.rightConeSensorId;

        double leftSensorValue = getSensorValueMillimeters(leftSensorId) / 1000.0;
        double rightSensorValue = getSensorValueMillimeters(rightSensorId) / 1000.0;
    
        distanceMeters =
            leftSensorValue
                + ((Constants.INTAKE.innerIntakeWidth + leftSensorValue - rightSensorValue) / 2) - (Constants.INTAKE.innerIntakeWidth / 2);
        break;
      case CUBE:
        int sensorId = Constants.INTAKE.cubeSensorId;

        double sensorValue = getSensorValueMillimeters(sensorId) / 1000.0;

        distanceMeters =
            sensorValue + (SimConstants.cubeWidth / 2) - (Constants.INTAKE.innerIntakeWidth / 2);
        break;
      default:
      case NONE:
        return 0;
    }

    return Units.metersToInches(distanceMeters);
  }

  // Returns a pose where the center of the gamepiece should be
  public Pose2d getGamepiecePose(INTAKING_STATES intakeState, Pose2d intakePose) {
    return new Pose2d(
        intakePose.getX(),
        intakePose.getY() + getGamepieceDistanceInches(intakeState),
        intakePose.getRotation());
  }

  public double getConeDistanceInches() {
    return getGamepieceDistanceInches(INTAKING_STATES.CONE);
  }

  public double getCubeDistanceInches() {
    return getGamepieceDistanceInches(INTAKING_STATES.CUBE);
  }

  public void pollDistanceSensors() {
    // This method will be called once per scheduler run
    // testParserTab.setInteger(testParser());
    try {
      if (RobotBase.isSimulation()) {
        simulationPeriodic();
      } else {
        byte[] buffer = new byte[512];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        socket.receive(packet);

        receivedData = new String(packet.getData(), 0, packet.getLength());
      }
      rawStringTab.setString(receivedData);
      sensorValue1MMTab.setDouble(getSensorValueMillimeters(1));
      sensorValue2MMTab.setDouble(getSensorValueMillimeters(2));
      sensorValue1InTab.setDouble(getSensorValueInches(1));
      sensorValue2InTab.setDouble(getSensorValueInches(2));
      coneInchesTab.setDouble(getConeDistanceInches());
      cubeInchesTab.setDouble(getCubeDistanceInches());
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
}
