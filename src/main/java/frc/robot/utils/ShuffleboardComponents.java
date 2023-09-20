package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DebugMode;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DebugMode.DebugPIDS;
import frc.robot.Constants.SwerveConstants.PIDConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

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
    private SimpleWidget visionRobotPoseX;
    private SimpleWidget visionRobotPoseY;
    private SimpleWidget visionFiducialID;
    private SimpleWidget visionLatency;

    //Debug Widgets
    private SimpleWidget debugMode;
    private SimpleWidget debugDriveP;
    private SimpleWidget debugDriveI;
    private SimpleWidget debugDriveD;
    private SimpleWidget debugSteerP;
    private SimpleWidget debugSteerI;
    private SimpleWidget debugSteerD;
    private SimpleWidget debugBalanceP;
    private SimpleWidget debugBalanceI;
    private SimpleWidget debugBalanceD;

    //Driver Widgets
    private SimpleWidget slowMode;
    private final SendableChooser<String> m_chooser;
    private SimpleWidget navXConnected;
    private SimpleWidget navXAngle;

    // Create Subsytems
    private Limelight limelight;
    private SwerveSubsystem swerveSubsystem;
    /**
     * Creates a new ShuffleboardComponents.
     */
    public ShuffleboardComponents() {
        // Create subsystems
        limelight = new Limelight();
        swerveSubsystem = new SwerveSubsystem();

        //Create tabs
        vision = Shuffleboard.getTab("Vision");
        driver = Shuffleboard.getTab("Driver");
        debug = Shuffleboard.getTab("Debug");
        swerve = Shuffleboard.getTab("Swerve");
        prematch = Shuffleboard.getTab("Prematch");

        // Create components
        m_chooser = new SendableChooser<>();

        //Vision
        usingVision = vision.add("Using Vision", false);
        visionTa = vision.add("Vision Ta", 0);
        visionTx = vision.add("Vision Tx", 0);
        visionTv = vision.add("Vision Tv", 0);
        visionRobotPoseX = vision.add("Vision Robot Pose", 0);
        visionRobotPoseY = vision.add("Vision Robot Pose", 0);
        visionFiducialID = vision.add("Vision Fiducial ID", 0);
        visionLatency = vision.add("Vision Latency", 0);

        //Driver
        slowMode = driver.add("Slow Mode", false);
        m_chooser.setDefaultOption("Low", "low");
        m_chooser.addOption("Middle", "middle");
        m_chooser.addOption("High", "high");
        driver.add("Scoring Level", m_chooser);
        navXConnected = swerve.add("NavX Connected", false);
        navXAngle = swerve.add("NavX Angle", 0);

        //Debug
        debugMode = debug.add("Debug Mode", false);
        debugDriveP = debug.add("Drive P", PIDConstants.DRIVE_PID.p);
        debugDriveI = debug.add("Drive I", PIDConstants.DRIVE_PID.i);
        debugDriveD = debug.add("Drive D", PIDConstants.DRIVE_PID.d);
        debugSteerP = debug.add("Steer P", PIDConstants.STEER_PID.p);
        debugSteerI = debug.add("Steer I", PIDConstants.STEER_PID.i);
        debugSteerD = debug.add("Steer D", PIDConstants.STEER_PID.d);
        debugBalanceP = debug.add("Balance P", PIDConstants.BALANCE_PID.p);
        debugBalanceI = debug.add("Balance I", PIDConstants.BALANCE_PID.i);
        debugBalanceD = debug.add("Balance D", PIDConstants.BALANCE_PID.d);

    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateVision();
        updateDriver();
        updateDebug();

    }

    /**
     * Updates the values displayed on the Vision tab.
     */
    private void updateVision() {
        SwerveConstants.usingVision = usingVision.getEntry().get().getBoolean();
        visionTa.getEntry().setDouble(limelight.getTa());
        visionTx.getEntry().setDouble(limelight.getTx());
        visionTv.getEntry().setDouble(limelight.getTv());
        visionRobotPoseX.getEntry().setDouble(limelight.getRobotPosition().getX());
        visionRobotPoseY.getEntry().setDouble(limelight.getRobotPosition().getY());
        visionFiducialID.getEntry().setDouble(limelight.getTag());
        visionLatency.getEntry().setDouble(limelight.getLatency());
    }

    private void updateDriver(){
        slowMode.getEntry().setBoolean(Constants.MotorConstants.SLOW_MODE);
        navXConnected.getEntry().setBoolean(swerveSubsystem.navXConnected());
        navXAngle.getEntry().setDouble(swerveSubsystem.getHeading());

    }

    private void updateDebug(){
        DebugMode.debugMode = debugMode.getEntry().get().getBoolean();
        DebugMode.DebugPIDS.debugDrive.p = debugDriveP.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugDrive.i = debugDriveI.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugDrive.d = debugDriveD.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugSteer.p = debugSteerP.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugSteer.i = debugSteerI.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugSteer.d = debugSteerD.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugBalance.p = debugBalanceP.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugBalance.i = debugBalanceI.getEntry().getDouble(0);
        DebugMode.DebugPIDS.debugBalance.d = debugBalanceD.getEntry().getDouble(0);
    }

    // Add other update methods for different tabs if needed

    public String getScoringLevel(){
        return m_chooser.getSelected();
    }
}
