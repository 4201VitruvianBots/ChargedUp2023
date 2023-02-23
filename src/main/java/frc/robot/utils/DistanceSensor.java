// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.UnknownHostException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensor {
  private final int socketPort = 25000;
  private final byte[] socketBuffer = new byte[85];

  private DatagramSocket socket;
  private String receivedData;

  /** Creates a new DistanceSensor. */
  public DistanceSensor() {
    try {
      InetAddress address = InetAddress.getByName("10.42.1.2"); //239.42.01.1
      socket = new DatagramSocket(socketPort, address);
    } catch (SocketException | UnknownHostException socketFail) {
      socketFail.printStackTrace();
    }
  }

  public String getRawSensorData() {
    return receivedData;
  }

  public int getSensorValue(int sensor) throws ParseException {
    String sensorName = "sensor" + Integer.toString(sensor) + ".mm";

    Object object = new JSONParser().parse(receivedData);
    JSONObject jsonObject = (JSONObject) object;
    return (int) jsonObject.get(sensorName);
  }

  public void periodic() {
    System.out.println("Periodic is running"); 
    // This method will be called once per scheduler run
    try {

      byte[] buffer = new byte[512];
      DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
      socket.receive(packet);

      receivedData = new String(packet.getData(), 0, packet.getLength());
      // System.out.println(receivedData);
      SmartDashboard.putString("Distance", receivedData); 
    } catch (SocketTimeoutException ex) {
      System.out.println("error: " + ex.getMessage());
      ex.printStackTrace();
    } catch (IOException ex) {
      System.out.println("Client error: " + ex.getMessage());
      ex.printStackTrace();
    }
    }
  }