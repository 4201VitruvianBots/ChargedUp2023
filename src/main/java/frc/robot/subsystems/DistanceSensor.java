// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.net.DatagramPacket;

public class DistanceSensor extends SubsystemBase {
  private final int socketPort = 25000;
  private final byte[] socketBuffer = new byte[85];

  private DatagramSocket socket;
  private String receivedData;

  /** Creates a new DistanceSensor. */
  public DistanceSensor() {
    try {
      socket = new DatagramSocket(socketPort);
    }
    catch (SocketException socketFail) {
      socketFail.printStackTrace();
    }
  }

  public String getRawSensorData() {
    return receivedData;
  }

  public int getSensorValue(int sensor) throws ParseException {
    String sensorName = "sensor"+Integer.toString(sensor)+".mm";

    Object object = new JSONParser().parse(receivedData);
    JSONObject jsonObject = (JSONObject) object;
    return (int) jsonObject.get(sensorName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    DatagramPacket packet 
    = new DatagramPacket(socketBuffer, socketBuffer.length);

    try {
      socket.receive(packet);
    } catch (IOException recieveFail) {
      recieveFail.printStackTrace();
    }
  
    InetAddress address = packet.getAddress();
    int port = packet.getPort();
    packet = new DatagramPacket(socketBuffer, socketBuffer.length, address, port);
    String receivedData
      = new String(packet.getData(), 0, packet.getLength());
    
    System.out.println(receivedData);
    socket.close();
  }
}
