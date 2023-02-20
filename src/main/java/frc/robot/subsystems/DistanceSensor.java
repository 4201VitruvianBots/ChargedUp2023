// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class DistanceSensor extends SubsystemBase {
  private final int socketPort = 25000;
  private final byte[] socketBuffer = new byte[85];

  private DatagramSocket socket;
  private String receivedData;

  /** Creates a new DistanceSensor. */
  public DistanceSensor() {
    try {
      socket = new DatagramSocket(socketPort);
    } catch (SocketException socketFail) {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    DatagramPacket packet = new DatagramPacket(socketBuffer, socketBuffer.length);

  //   try { 
  //     InetAddress address = InetAddress.getByName("239, 42, 01, 1");
  //     DatagramSocket socket = new DatagramSocket();
  //      // change this js line to java 
  //     setTimeout(() -> {console.log("this is the first message");}, 1000); TODO: Fix setTimeout because it can't be found
        
  //         byte[] buffer = new byte[512];
  //         DatagramPacket response = new DatagramPacket(buffer, buffer.length);
  //         socket.receive(response);

  //         String quote = new String(buffer, 0, response.getLength());

  //         System.out.println(quote);
  //         System.out.println();

  //         Thread.sleep(10000);

  // } catch (SocketTimeoutException ex) {
  //   System.out.println("                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           error: " + ex.getMessage());
  //     ex.printStackTrace();
  // } catch (IOException ex) {
  //     System.out.println("Client error: " + ex.getMessage());
  //     ex.printStackTrace();
  // } catch (InterruptedException ex) {
  //     ex.printStackTrace();
  // }

    InetAddress address = packet.getAddress();
    int port = packet.getPort();
    packet = new DatagramPacket(socketBuffer, socketBuffer.length, address, port);
    receivedData = new String(packet.getData(), 0, packet.getLength());

    System.out.println(receivedData);
    socket.close();
  }

  private void setTimeout() {
  }
}
