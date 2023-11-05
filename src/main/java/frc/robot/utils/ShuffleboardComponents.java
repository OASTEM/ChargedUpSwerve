package frc.robot.utils;

import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DebugMode;
import frc.robot.Constants.MotorConstants;
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

    //Vision Widgets
    private SimpleWidget usingVision;
    private SimpleWidget visionTx;
    private SimpleWidget visionTy;
    private SimpleWidget visionTa;
    private SimpleWidget visionTv;
    private SimpleWidget visionRobotPoseX;
    private SimpleWidget visionRobotPoseY;
    private SimpleWidget visionFiducialID;
    private SimpleWidget visionLatency;
    private SimpleWidget visionTargetPoseX;
    private SimpleWidget visionTargetPoseY;

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
    private SimpleWidget canCoder9;
    private SimpleWidget canCoder10;
    private SimpleWidget canCoder11;
    private SimpleWidget canCoder12;
    private SimpleWidget xpos;
    private SimpleWidget ypos;
    private SimpleWidget autoXP;
    private SimpleWidget autoXI;
    private SimpleWidget autoXD;
    private SimpleWidget autoYP;
    private SimpleWidget autoYI;
    private SimpleWidget autoYD;
    private SimpleWidget autoRotationP;
    private SimpleWidget autoRotationI;
    private SimpleWidget autoRotationD;
    private SimpleWidget rotorOnePosition;
    private SimpleWidget rotorTwoPosition;
    private SimpleWidget rotorThreePosition;
    private SimpleWidget rotorFourPosition;
    private SimpleWidget fusedHeading;
    private SimpleWidget robotHeading;

    //Driver Widgets
    private SimpleWidget slowMode;
    private SimpleWidget aacornMode;
    private SimpleWidget navXConnected;
    private SimpleWidget navXCalibrating;
    private SimpleWidget pitch;
    private SimpleWidget roll;
    private SimpleWidget yaw;
    private SimpleWidget compassHeading;
    private SimpleWidget rightRotation;
    private SimpleWidget heading;
    private SimpleWidget desiredAngle;
    private SimpleWidget desiredAngleSpeed;
    private SimpleWidget computedAngleSpeed;
    
    /**
     * Creates a new ShuffleboardComponents.
     */
    private SwerveSubsystem swerveSubsystem;
    private Limelight limelight; 
    
    public ShuffleboardComponents(SwerveSubsystem swerveSubsystem, Limelight limelight) {
        // Create subsystems
        this.swerveSubsystem = swerveSubsystem;
        this.limelight = limelight;

        //Create tabs
        vision = Shuffleboard.getTab("Vision");
        driver = Shuffleboard.getTab("Driver");
        debug = Shuffleboard.getTab("Debug");

        // Create components

        //Vision
        usingVision = vision.add("Using Vision", false);
        visionTa = vision.add("Vision Ta", 0);
        visionTx = vision.add("Vision Tx", 0);
        visionTy = vision.add("Vision Ty", 0);
        visionTv = vision.add("Vision Tv", 0);
        visionRobotPoseX = vision.add("Vision Robot Pose X", 0);
        visionRobotPoseY = vision.add("Vision Robot Pose Y", 0);
        visionFiducialID = vision.add("Vision Fiducial ID", 0);
        visionLatency = vision.add("Vision Latency", 0);
        visionTargetPoseX = vision.add("Vision Target Pose X", 0);
        visionTargetPoseY = vision.add("Vision Target Pose Y", 0);

        //Driver
        rightRotation = driver.add("Rotation", 0);
        heading = driver.add("Robot Heading", 0);
        slowMode = driver.add("Slow Mode", false);
        aacornMode = driver.add("Aacorn Mode", false);
        desiredAngle = driver.add("Desired Angle", 0);
        desiredAngleSpeed = driver.add("Desired Angle Speed", 0);
        computedAngleSpeed = driver.add("Computed Speed", MotorConstants.computedAngleSpeed);
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
        navXConnected = debug.add("NavX Connected", false);
        navXCalibrating = debug.add("NavX Calibrating", false);
        pitch = debug.add("Pitch", 0);
        roll = debug.add("Roll", 0);
        yaw = debug.add("Yaw", 0);
        fusedHeading = debug.add("Fused Heading", 0);
        compassHeading = debug.add("Compass Heading", 0);
        canCoder9 = debug.add("CanCoder 9", 0);
        canCoder10 = debug.add("CanCoder 10", 0);
        canCoder11 = debug.add("CanCoder 11", 0);
        canCoder12 = debug.add("CanCoder 12", 0);
        xpos = debug.add("X position", 0);
        ypos = debug.add("Y position", 0);
        autoXP = debug.add("AutoX P", PIDConstants.AUTO_X.p);
        autoXI = debug.add("AutoX I", PIDConstants.AUTO_X.i);
        autoXD = debug.add("AutoX D", PIDConstants.AUTO_X.d);
        autoYP = debug.add("AutoY P", PIDConstants.AUTO_Y.p);
        autoYI = debug.add("AutoY I", PIDConstants.AUTO_Y.i);
        autoYD = debug.add("AutoY D", PIDConstants.AUTO_Y.d);
        autoRotationP = debug.add("AutoRotation P", PIDConstants.AUTO_ROTATION.p);
        autoRotationI = debug.add("AutoRotation I", PIDConstants.AUTO_ROTATION.i);
        autoRotationD = debug.add("AutoRotation D", PIDConstants.AUTO_ROTATION.d);
        rotorOnePosition = debug.add("Rotor One Position", swerveSubsystem.getRotorPositions(0));
        rotorTwoPosition = debug.add("Rotor Two Position", swerveSubsystem.getRotorPositions(1));
        rotorThreePosition = debug.add("Rotor Three Position", swerveSubsystem.getRotorPositions(2));
        rotorFourPosition = debug.add("Rotor Four Position", swerveSubsystem.getRotorPositions(3));
        robotHeading = debug.add("Robot Heading", swerveSubsystem.pgetHeading());

        //REMOVE ALL THIS JAYDEN SUN DID THIS

    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateVision();
        updateDriver();
        // updat 8/eDxx[['ebug();
    }

    /**
     * Updates the values displayed on the Vision tab.
     */
    private void updateVision() {
        if (limelight.getTx() > 0){
            visionFiducialID.getEntry().setDouble(limelight.getTag());
            visionTa.getEntry().setDouble(limelight.getTa());
            visionTx.getEntry().setDouble(limelight.getTx());
            visionTy.getEntry().setDouble(limelight.getTy());
            visionTv.getEntry().setDouble(limelight.getTv());
            visionRobotPoseX.getEntry().setDouble(limelight.getRobotPosition().getX());
            visionRobotPoseY.getEntry().setDouble(limelight.getRobotPosition().getY());
            visionFiducialID.getEntry().setDouble(limelight.getTag());
            visionLatency.getEntry().setDouble(limelight.getLatency());
            usingVision.getEntry().setBoolean(SwerveConstants.usingVision);
        }
    }

    private void updateDriver(){
        slowMode.getEntry().setBoolean(Constants.MotorConstants.SLOW_MODE);
        aacornMode.getEntry().setBoolean(Constants.MotorConstants.AACORN_MODE);
        pitch.getEntry().setDouble(swerveSubsystem.getPitch());
        yaw.getEntry().setDouble(swerveSubsystem.getYaw());
        // fusedHeading.getEntry().setDouble(swerveSubsystem.getHeading());
        canCoder9.getEntry().setDouble(swerveSubsystem.getCanCoderValues(MotorConstants.FRONT_LEFT_CAN_CODER_ID));
        canCoder10.getEntry().setDouble(swerveSubsystem.getCanCoderValues(MotorConstants.FRONT_RIGHT_CAN_CODER_ID));
        canCoder11.getEntry().setDouble(swerveSubsystem.getCanCoderValues(MotorConstants.BACK_LEFT_CAN_CODER_ID));
        canCoder12.getEntry().setDouble(swerveSubsystem.getCanCoderValues(MotorConstants.BACK_RIGHT_CAN_CODER_ID));
        heading.getEntry().setDouble(swerveSubsystem.pgetHeading());
        desiredAngle.getEntry().setDouble(MotorConstants.desiredAngle);
        desiredAngleSpeed.getEntry().setDouble(MotorConstants.desiredAngleSpeed);
        computedAngleSpeed.getEntry().setDouble(MotorConstants.computedAngleSpeed);
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

        SwerveConstants.PIDConstants.AUTO_X.p = autoXP.getEntry().getDouble(0);
        SwerveConstants.PIDConstants.AUTO_X.i = autoXI.getEntry().getDouble(0);
        SwerveConstants.PIDConstants.AUTO_X.d = autoXD.getEntry().getDouble(0);

        SwerveConstants.PIDConstants.AUTO_Y.p = autoYP.getEntry().getDouble(0);
        SwerveConstants.PIDConstants.AUTO_Y.i = autoYI.getEntry().getDouble(0);
        SwerveConstants.PIDConstants.AUTO_Y.d = autoYD.getEntry().getDouble(0);

        SwerveConstants.PIDConstants.AUTO_ROTATION.p = autoRotationP.getEntry().getDouble(0);
        SwerveConstants.PIDConstants.AUTO_ROTATION.i = autoRotationI.getEntry().getDouble(0);
        SwerveConstants.PIDConstants.AUTO_ROTATION.d = autoRotationD.getEntry().getDouble(0);

        rotorOnePosition.getEntry().setDouble(swerveSubsystem.getRotorPositions(0));
        rotorTwoPosition.getEntry().setDouble(swerveSubsystem.getRotorPositions(1));
        rotorThreePosition.getEntry().setDouble(swerveSubsystem.getRotorPositions(2));
        rotorFourPosition.getEntry().setDouble(swerveSubsystem.getRotorPositions(3));

        xpos.getEntry().setDouble(swerveSubsystem.getX());
        ypos.getEntry().setDouble(swerveSubsystem.getY());

        robotHeading.getEntry().setDouble(swerveSubsystem.pgetHeading());
    }

    // Add other update methods for different tabs if needed
}


