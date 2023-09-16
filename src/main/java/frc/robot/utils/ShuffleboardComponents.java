package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BalanceDebug;
import frc.robot.Constants.DebugMode;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Limelight;

public class ShuffleboardComponents extends SubsystemBase {
    // Create tabs
    private ShuffleboardTab vision;
    private ShuffleboardTab driver;
    private ShuffleboardTab debug;
    private ShuffleboardTab swerve;
    private ShuffleboardTab prematch;

    //Vision Widgets
    private SimpleWidget usingVision;
    private SimpleWidget visionTx;
    private SimpleWidget visionTa;
    private SimpleWidget visionTv;

    //Debug Widgets
    private SimpleWidget debugMode;
    private SimpleWidget debugDriveP;
    private SimpleWidget debugDriveI;
    private SimpleWidget debugDriveD;
    private SimpleWidget debugSteerP;
    private SimpleWidget debugSteerI;
    private SimpleWidget debugSteerD;

    //Driver Widgets
    private SimpleWidget slowMode;

    //Swerve Widgets
    private SimpleWidget steerP;
    private SimpleWidget driveP;
    private SimpleWidget steerI;
    private SimpleWidget driveI;
    private SimpleWidget steerD;
    private SimpleWidget driveD;

    // Create Subsytems
    private Limelight limelight;
    /**
     * Creates a new ShuffleboardComponents.
     */
    public ShuffleboardComponents() {
        vision = Shuffleboard.getTab("Vision");
        driver = Shuffleboard.getTab("Driver");
        debug = Shuffleboard.getTab("Debug");
        swerve = Shuffleboard.getTab("Swerve");
        prematch = Shuffleboard.getTab("Prematch");

        // Create components

        //Vision
        usingVision = vision.add("Using Vision", false);
        visionTa = vision.add("Vision Ta", 0);
        visionTx = vision.add("Vision Tx", 0);
        visionTv = vision.add("Vision Tv", 0);

        //Driver
        slowMode = driver.add("Slow Mode", false);

        //Debug
        debugMode = debug.add("Debug Mode", false);
        debugDriveP = debug.add("Drive P", 0);
        debugDriveI = debug.add("Drive I", 0);
        debugDriveD = debug.add("Drive D", 0);
        debugSteerP = debug.add("Steer P", 0);
        debugSteerI = debug.add("Steer I", 0);
        debugSteerD = debug.add("Steer D", 0);
        
        //Swerve

        //Prematch
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateVision();
        updateDriver();
        updateDebug();
        updateSwerve();
        updatePrematch();

        // System.out.println(SwerveConstants.usingVision);
    }

    /**
     * Updates the values displayed on the Vision tab.
     */
    private void updateVision() {
        SwerveConstants.usingVision = usingVision.getEntry().get().getBoolean();
        visionTa.getEntry().setDouble(limelight.getTa());
        visionTx.getEntry().setDouble(limelight.getTx());
        visionTv.getEntry().setDouble(limelight.getTv());
    }

    private void updateDriver(){
        slowMode.getEntry().setBoolean(Constants.MotorConstants.SLOW_MODE);
    }

    private void updateDebug(){
        DebugMode.debugMode = debugMode.getEntry().get().getBoolean();
        DebugMode.DebugPIDS.debugDrive.p = debugDriveP.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugDrive.i = debugDriveI.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugDrive.d = debugDriveD.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugSteer.p = debugSteerP.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugSteer.i = debugSteerI.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugSteer.d = debugSteerD.getEntry().getDouble(0);
    }

    private void updateSwerve(){

    }

    private void updatePrematch(){
        
    }

    // Add other update methods for different tabs if needed
}
