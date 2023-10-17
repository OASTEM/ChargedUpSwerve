package frc.robot.subsystems;

import java.util.function.Consumer;

import com.fasterxml.jackson.databind.deser.impl.SetterlessProperty;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
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
  private double delta;
  private final Limelight limelight;
  private Rotation2d gyroAngle;
  // private final AHRS navX = new AHRS(SPI.Port.kMXP);
  // private final AHRS navX = new AHRS(SerialPort.Port.kUSB1);
  private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);

  private boolean vision;
  private static double printSlow = 0;
  private final SwerveDriveKinematics sKinematics = Constants.SwerveConstants.kinematics;
  private double JaydenSun = 0;
  private Timer timer;
  /** Creates a new DriveTrain. */
  public SwerveSubsystem(boolean vision, Limelight limelight) {

    // navX.reset();
    // frontLeft = new SwerveModule(Constants.MotorConstants.frontLeftDriveId,
    // Constants.MotorConstants.frontLeftSteerId);
    // frontRight = new SwerveModule(Constants.MotorConstants.frontRightDriveId,
    // Constants.MotorConstants.frontRightSteerId);
    // backLeft = new SwerveModule(Constants.MotorConstants.backLeftDriveId,
    // Constants.MotorConstants.backLeftSteerId);
    // backRight = new SwerveModule(Constants.MotorConstants.backRightDriveId,
    // Constants.MotorConstants.backRightSteerId);
    this.vision = vision;
    this.limelight = limelight;
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
    addRotorPositionsforModules();
    navX.calibrate();
    MotorConstants.desiredAngle = getHeading();
  }

  public void drive(double forwardSpeed, double leftSpeed, double joyStickInput, boolean isFieldOriented) {
    ChassisSpeeds speeds;

    MotorConstants.desiredAngleSpeed = joyStickInput * Constants.MotorConstants.TURN_CONSTANT;
    MotorConstants.desiredAngle += MotorConstants.desiredAngleSpeed / 50;
    // MotorConstants.computedAngleSpeed = MotorConstants.desiredAngleSpeed - (Math.toRadians(getHeading()) - MotorConstants.desiredAngle);
    delta = MotorConstants.desiredAngle - Math.toRadians(getHeading());
    MotorConstants.computedAngleSpeed = delta * -0.02;
    
    JaydenSun = joyStickInput * Constants.MotorConstants.TURN_CONSTANT;

    if (isFieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          forwardSpeed,
          leftSpeed,
          JaydenSun,
          getRotation2d());
    } else {
      speeds = new ChassisSpeeds(
          forwardSpeed,
          leftSpeed,
          joyStickInput);
    }

    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
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
    navX.getRotation2d();
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
<<<<<<< Updated upstream
    // System.out.println(navX.getAngle() % 360);
    return navX.getFusedHeading() % 360;
=======
    // return navX.getAngle() % 360;
    return navX.getRotation2d().getDegrees() % 360;
>>>>>>> Stashed changes
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

  public double getCompassHeading(){
    return navX.getCompassHeading();
  }

  public double findCompassYaw(){
    return navX.getAngle()-navX.getCompassHeading();
  }
  public void configSlowMode() {
    Constants.MotorConstants.SLOW_MODE = !Constants.MotorConstants.SLOW_MODE;
  }

  public boolean getSlowMode(){
    return Constants.MotorConstants.SLOW_MODE;
  }

  public void configAAcornMode() { 
    Constants.MotorConstants.AACORN_MODE = !Constants.MotorConstants.AACORN_MODE;
  }

  public boolean getAAcornMode(){
    return Constants.MotorConstants.AACORN_MODE;
  }

  /** Gets the NavX angle as a Rotation2d. */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
    // return Rotation2d.fromDegrees(0);
  }

  public void updatePosition(){
    this.addVision(limelight.getRobotPosition());
  }

  @Override
  public void periodic() {
      // This method will be called once per scheduler run
      
      if (vision)
      {
        updatePosition();
      }

    gyroAngle = Rotation2d.fromDegrees(getYaw());
    estimator.update(gyroAngle, getModulePositions());
    // System.out.print("Pitch" + getPitch() + " ");
    // System.out.print("Roll" + getRoll() + " ");
    // System.out.println("Yaw" + getYaw());
    // for (int i = 0; i < modules.length; i++) {
    //   modules[i].setDebugPID(Constants.DebugMode.debugMode);
    // }
  }

  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
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
    estimator.update(getRotation2d(), getModulePositions());
    
  }
<<<<<<< Updated upstream
  public void addVision(){
    estimator.addVisionMeasurement(null, Timer.getFPGATimestamp());
  }
=======
>>>>>>> Stashed changes

  public Pose2d getOdometry(){
    return estimator.getEstimatedPosition();
  }

  public void driveStraight(double speed){
    this.drive(speed, 0.0, 0.0, false);
  }

// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
       new InstantCommand(() -> {
         // Reset odometry for the first path you run during auto
         if(isFirstPath){
             this.zeroHeading();
             System.out.println(isFirstPath);
         }
       }),
       new PPSwerveControllerCommand(
           traj,
           this::getOdometry, // Pose supplier
           this.sKinematics, // SwerveDriveKinematics
           new PIDController(SwerveConstants.PIDConstants.AUTO_X.p, SwerveConstants.PIDConstants.AUTO_X.i, SwerveConstants.PIDConstants.AUTO_X.d), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(SwerveConstants.PIDConstants.AUTO_Y.p, SwerveConstants.PIDConstants.AUTO_Y.i, SwerveConstants.PIDConstants.AUTO_Y.d), // Y controller (usually the same values as X controller)
           new PIDController(SwerveConstants.PIDConstants.AUTO_ROTATION.p, SwerveConstants.PIDConstants.AUTO_ROTATION.i, SwerveConstants.PIDConstants.AUTO_ROTATION.d), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           this::outputModuleStates, // Module states consumer
           false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           this // Requires this drive subsqystem
       )
   );
}

  public void addVision(Pose2d visionPose){
    estimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
  }
  
  public void outputModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Constants.MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public double getRotorPositions(int moduleNum){
    return modules[moduleNum].steerMotor.getRotorPosition().getValue();
  }

  

  public boolean navXConnected(){
    return navX.isConnected();
  }

  public boolean navXCalibrating() {
    return navX.isCalibrating();
  }

  public double getCanCoderValues(int canID){
    double[] canCoderValues = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      canCoderValues[i] = modules[i].getCanCoderValue();
    }
    return canCoderValues[canID-9];
  }

  public double getX(){
    return estimator.getEstimatedPosition().getTranslation().getX();
  }

  public double getY(){
    return estimator.getEstimatedPosition().getTranslation().getY();
  }

}


