// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.StringReader;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensor {
  private final int socketPort = 25000;
  private final byte[] socketBuffer = new byte[85];

  private DatagramSocket socket;
  private String receivedData;

  private final String mainPath = new File("").getAbsolutePath();
  private final String testPath = mainPath + "/resources/sensorReading.json";

  // Shuffleboard setup

  public static ShuffleboardTab distanceSensorTab = Shuffleboard.getTab("DistanceSensor");

  public GenericEntry rawStringTab = distanceSensorTab.add("Raw String Data", "None").getEntry();
  public GenericEntry sensorValue1Tab = distanceSensorTab.add("Sensor Value 1", 0).getEntry();
  public GenericEntry sensorValue2Tab = distanceSensorTab.add("Sensor Value 2", 0).getEntry();
  public GenericEntry testParserTab = distanceSensorTab.add("Test Value 1", 0).getEntry();
  
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

  public int getSensorValue(int sensor) {
    try {
      String sensorName = "sensor" + Integer.toString(sensor) + ".mm";

      // parsing string from recieved data
      Object obj = new JSONParser().parse(new StringReader(receivedData));

      // typecasting obj to JSONObject
      JSONObject jo = (JSONObject) obj;

      // getting sensor value
      long sensorValueLong = (long) jo.get(sensorName);

      // auto-unboxing does not go from Long to int directly, so
      int sensorValue = (int) (long) sensorValueLong;

      return sensorValue;
    } catch (Exception e) {
      System.out.println("Boo hoo I can't read the file :_(");
      e.printStackTrace();
      return -1;
    }
  }

  public int testParser() {
    try {
      String sensorName = "sensor1.mm";

      // parsing file from resources
      Object obj = new JSONParser().parse(new FileReader(testPath));

      // typecasting obj to JSONObject
      JSONObject jo = (JSONObject) obj;

      // getting sensor value
      long sensorValueLong = (long) jo.get(sensorName);

      // auto-unboxing does not go from Long to int directly, so
      int sensorValue = (int) (long) sensorValueLong;

      return sensorValue;
    } catch (Exception e) {
      System.out.println("Boo hoo I can't read the file :_(");
      e.printStackTrace();
      return -1;
    }
  }

  public void pollDistanceSensors() {
      // This method will be called once per scheduler run
      testParserTab.setInteger(testParser());
      try {

        byte[] buffer = new byte[512];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        socket.receive(packet);

        receivedData = new String(packet.getData(), 0, packet.getLength());

        rawStringTab.setString(receivedData);
        sensorValue1Tab.setInteger(getSensorValue(1));
        sensorValue2Tab.setInteger(getSensorValue(2));
        //SmartDashboard.putString("Is running", "Is running");
        //SmartDashboard.putString("Distance", receivedData);
        //SmartDashboard.putNumber("Parsed Sensor 1", getSensorValue(1));
        //SmartDashboard.putNumber("Parsed Sensor 2", getSensorValue(2));
        //System.out.println(getSensorValue(1));
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
