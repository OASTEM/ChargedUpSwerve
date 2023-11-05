package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveConstants;

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
  // private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);
  private Pigeon2 pidggy;

  private double rot;
  private boolean vision;
  private final SwerveDriveKinematics sKinematics = Constants.SwerveConstants.kinematics;
  private double turnSpeed = 0;
  private Timer timer;
  /** Creates a new DriveTrain. */
  public SwerveSubsystem(boolean vision, Limelight limelight) {

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

    pidggy = new Pigeon2(16);

    // Creating my odometry object from the kinematics object and the initial wheel
    // positions.
    // Here, our starting pose is 5 meters along the long end of the field and in
    // the
    // center of the field along the short end, facing the opposing alliance wall.
    estimator = new SwerveDrivePoseEstimator(
        SwerveConstants.kinematics,
        getRotationPidggy(),
        getModulePositions(),
        SwerveConstants.STARTING_POSE
        );

    addRotorPositionsforModules();
    MotorConstants.desiredAngle = pgetHeading();
    timer = new Timer();
  }

  public void drive(double forwardSpeed, double leftSpeed, double joyStickInput, boolean isFieldOriented) {
    ChassisSpeeds speeds;

    MotorConstants.desiredAngleSpeed = joyStickInput * Constants.MotorConstants.TURN_CONSTANT;
    MotorConstants.desiredAngle += MotorConstants.desiredAngleSpeed / 50;
    // MotorConstants.computedAngleSpeed = MotorConstants.desiredAngleSpeed - (Math.toRadians(getpheading()) - MotorConstants.desiredAngle);
    delta = MotorConstants.desiredAngle - Math.toRadians(pgetHeading());
    MotorConstants.computedAngleSpeed = delta * -0.02;
    
    turnSpeed = joyStickInput * Constants.MotorConstants.TURN_CONSTANT;

    if (isFieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          forwardSpeed,
          leftSpeed,
          turnSpeed,
          getRotationPidggy());
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

  public Rotation2d getRotationPidggy(){
    rot = -pidggy.getRotation2d().getDegrees();
    return Rotation2d.fromDegrees(rot);
    // SmartDashboard.putNumber("Rotation2d", pidggy.getRotation2d().getDegrees());
    // return pidggy.getRotation2d();
  }

  public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d newPose) {
    estimator.resetPosition(gyroAngle, modulePositions, newPose);
    // navX.getRotation2d();
  }

  public void zeroHeading() {
    pidggy.reset();
  }

  public void heading180() {
    pidggy.setYaw(180);
  }

  // public void resetPitch(){
  //   pidggy.setYawPitchRoll(0,0,0);
  // }

  public void resetDriveEncoders() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].resetEncoders();
    }
  }
  public double getYaw(){
    return pidggy.getYaw().getValue();
  }

  public double pgetHeading(){
    return(pidggy.getYaw().getValue() % 360);
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

  // public Rotation2d getRotation2d() {
  //   return Rotation2d.fromDegrees(pidggy.getYaw().getValue());
  //   // return Rotation2d.fromDegrees(0);
  // }

  public void updatePosition(){
    this.addVision(limelight.getRobotPosition());
  }

  @Override
  public void periodic() {
      // This method will be called once per scheduler run
      pidggy.getYaw().refresh();
      if (vision)
      {
        updatePosition();
      }

    gyroAngle = getRotationPidggy();
    estimator.update(gyroAngle, getModulePositions());
    // System.out.print("Pitch" + getPitch() + " ");
    // System.out.print("Roll" + getRoll() + " ");
    // System.out.println("Yaw" + getYaw());
    // for (int i = 0; i < modules.length; i++) {
    //   modules[i].setDebugPID(Constants.DebugMode.debugMode);
    // }
    // SmartDashboard.putNumber("NavXXXX", navX.getFusedHeading());
  }

  public double getPitch(){
    return pidggy.getPitch().getValue();
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
    estimator.update(getRotationPidggy(), getModulePositions());
    
  }
  public void addVision(){
    estimator.addVisionMeasurement(null, Timer.getFPGATimestamp());
  }

  public Pose2d getOdometry(){
    return estimator.getEstimatedPosition();
  }

  public void driveStraight(double speed){
    this.drive(speed, 0.0, 0.0, true);
  }

  public void driveSide(double speed){
    this.drive(0.0, speed, 0.0, true);
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


