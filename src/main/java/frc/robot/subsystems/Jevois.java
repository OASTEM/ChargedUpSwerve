// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Jevois extends SubsystemBase {
  /** Creates a new Jevois. */
  private static final int BAUD = 9600;
  private SerialPort camPort;
  private UsbCamera jevois;
  
  public Jevois() {

    jevois = new UsbCamera("cam0", 1); 
    jevois = CameraServer.startAutomaticCapture();
    jevois.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 240, 30);

    try{
      System.out.println("1st Try");
      camPort = new SerialPort(BAUD, SerialPort.Port.kUSB);
    } catch (Exception e){
      System.out.println("Error - 2nd Try");
      try{
        camPort = new SerialPort(BAUD, SerialPort.Port.kUSB1);
      } catch (Exception f){
        try{
          System.out.println("Error - 3rd Trye");
          camPort = new SerialPort(BAUD, SerialPort.Port.kUSB2);  
        } catch (Exception g){
          System.out.println("Could not connect jevois with robot");
        }
      } 
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
