package frc.robot.subsystems;

import java.util.function.Consumer;

import com.fasterxml.jackson.databind.deser.impl.SetterlessProperty;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import frc.robot.utils.LimelightHelpers;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {
  // private SwerveModule frontLeft;
  // private SwerveModule frontRight;
  // private SwerveModule backLeft;
  // private SwerveModule backRight;
  private SwerveModule[] modules;
  private SwerveDrivePoseEstimator estimator;
  // private final AHRS navX = new AHRS(SPI.Port.kMXP);
  // private final AHRS navX = new AHRS(SerialPort.Port.kUSB1);
  private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);

  private static double printSlow = 0;
  private final SwerveDriveKinematics sKinematics = Constants.SwerveConstants.kinematics;

  /** Creates a new DriveTrain. */
  public SwerveSubsystem() {
    // navX.reset();
    // frontLeft = new SwerveModule(Constants.MotorConstants.frontLeftDriveId,
    // Constants.MotorConstants.frontLeftSteerId);
    // frontRight = new SwerveModule(Constants.MotorConstants.frontRightDriveId,
    // Constants.MotorConstants.frontRightSteerId);
    // backLeft = new SwerveModule(Constants.MotorConstants.backLeftDriveId,
    // Constants.MotorConstants.backLeftSteerId);
    // backRight = new SwerveModule(Constants.MotorConstants.backRightDriveId,
    // Constants.MotorConstants.backRightSteerId);

    modules = new SwerveModule[] {
        new SwerveModule(
            MotorConstants.FRONT_LEFT_DRIVE_ID,
            MotorConstants.FRONT_LEFT_STEER_ID,
            MotorConstants.FRONT_LEFT_CAN_CODER_ID,
            Constants.SwerveConstants.CANCoderValue9),
        new SwerveModule(
            MotorConstants.FRONT_RIGHT_DRIVE_ID,
            MotorConstants.FRONT_RIGHT_STEER_ID,
            MotorConstants.FRONT_RIGHT_CAN_CODER_ID,
            Constants.SwerveConstants.CANCoderValue10),
        new SwerveModule(
            MotorConstants.BACK_LEFT_DRIVE_ID,
            MotorConstants.BACK_LEFT_STEER_ID,
            MotorConstants.BACK_LEFT_CAN_CODER_ID,
            Constants.SwerveConstants.CANCoderValue11),
        new SwerveModule(
            MotorConstants.BACK_RIGHT_DRIVE_ID,
            MotorConstants.BACK_RIGHT_STEER_ID,
            MotorConstants.BACK_RIGHT_CAN_CODER_ID,
            Constants.SwerveConstants.CANCoderValue12)
    };

    // Creating my odometry object from the kinematics object and the initial wheel
    // positions.
    // Here, our starting pose is 5 meters along the long end of the field and in
    // the
    // center of the field along the short end, facing the opposing alliance wall.
    estimator = new SwerveDrivePoseEstimator(
        SwerveConstants.kinematics,
        getRotation2d(),
        getModulePositions(),
        SwerveConstants.STARTING_POSE
        );

    // new Thread(() -> {
    // try {
    // Thread.sleep(1000);
    // zeroHeading();
    // } catch (Exception e) {
    // }
    // }).start();
  }

  public void drive(double forwardSpeed,
      double leftSpeed, double rotationSpeed, boolean isFieldOriented) {
    ChassisSpeeds speeds;


    if (isFieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          forwardSpeed,
          leftSpeed,
          rotationSpeed,
          Rotation2d.fromDegrees(navX.getAngle()));
      // Rotation2d.fromDegrees(0));
    } else {
      speeds = new ChassisSpeeds(
          forwardSpeed,
          leftSpeed,
          rotationSpeed);
    }

    // System.out.println(speeds);

    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
    // System.out.println(states);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);

      // System.out.println(states[i]);
      // System.out.println(i);
    }

    updateAllSteerPositionSmartDashboard();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d newPose) {
    estimator.resetPosition(gyroAngle, modulePositions, newPose);
  }

  public void zeroHeading() {
    navX.reset();
  }

  public void resetDriveEncoders() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].resetEncoders();
    }
  }

  public double getHeading() {
    return navX.getAngle() % 360;
  }

  public double getPitch() {
    return navX.getPitch();
  }

  public double getYaw() {
    return navX.getYaw();
  }

  public double getRoll() {
    return navX.getRoll();
  }

  public void configSlowMode() {
    Constants.MotorConstants.SLOW_MODE = !Constants.MotorConstants.SLOW_MODE;
  }

  public boolean getSlowMode(){
    return Constants.MotorConstants.SLOW_MODE;
  }

  /** Gets the NavX angle as a Rotation2d. */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
    // return Rotation2d.fromDegrees(0);
  }

  @Override
  public void periodic() {
      // This method will be called once per scheduler run

    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putBoolean("Is Connected", navX.isConnected());

    SmartDashboard.putNumber("Yaw", getYaw());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Roll", getRoll());

    Rotation2d gyroAngle = getRotation2d();
    estimator.update(gyroAngle, getModulePositions());
  }

  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  public void updateAllSteerPositionSmartDashboard() {
    for (SwerveModule currModule : modules) {
      currModule.updateSteerPositionSmartDashboard();
    }
  }

  public void test(int moduleNum, double driveSpeed, double rotationSpeed) {
    SwerveModule module = modules[moduleNum];

    module.setDriveSpeed(driveSpeed);
    module.setSteerSpeed(rotationSpeed);
  }

  public void addRotorPositionsforModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setRotorPos();
    }
  }

  public void stop() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].stop();
    }
  }


  public void updateEstimator(){
    // estimator.update(getRotation2d(), getModulePositions());
    
  }
  public void addVision(){
    // estimator.addVisionMeasurement(null, Timer.getFPGATimestamp());
  }

  public Pose2d getOdometry(){
    return estimator.getEstimatedPosition();
  }

// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
       new InstantCommand(() -> {
         // Reset odometry for the first path you run during auto
         if(isFirstPath){
             this.zeroHeading();
         }
       }),
       new PPSwerveControllerCommand(
           traj, 
           this::getOdometry, // Pose supplier
           this.sKinematics, // SwerveDriveKinematics
           new PIDController(0.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0.1, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0.1, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           this::outputModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           this // Requires this drive subsystem
       )
   );
}

  public void addVision(Pose2d visionPose){
    estimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
    SmartDashboard.putNumber("X field", visionPose.getX());
    SmartDashboard.putNumber("Y field", visionPose.getY());
  }
  
  public void outputModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);

      // System.out.println(states[i]);
      // System.out.println(i);
    }
  }
}