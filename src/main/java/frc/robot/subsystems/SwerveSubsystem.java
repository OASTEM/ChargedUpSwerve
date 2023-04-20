// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {
  // private SwerveModule frontLeft;
  // private SwerveModule frontRight;
  // private SwerveModule backLeft;
  // private SwerveModule backRight;
  private SwerveModule[] modules;
  private SwerveDriveOdometry odometry;

  private final AHRS navX = new AHRS(SPI.Port.kMXP);

  /** Creates a new DriveTrain. */
  public SwerveSubsystem() {
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
            MotorConstants.FRONT_LEFT_STEER_ID),
        new SwerveModule(
            MotorConstants.FRONT_RIGHT_DRIVE_ID,
            MotorConstants.FRONT_RIGHT_STEER_ID),
        new SwerveModule(
            MotorConstants.BACK_LEFT_DRIVE_ID,
            MotorConstants.BACK_LEFT_STEER_ID),
        new SwerveModule(
            MotorConstants.BACK_RIGHT_DRIVE_ID,
            MotorConstants.BACK_RIGHT_STEER_ID)
    };

    SwerveModulePosition[] initPositions = {}

    odometry = new SwerveDriveOdometry(
      SwerveConstants.kinematics, getHeading(), new SwerveModulePosition[] {}
    );

    // new Thread(() -> {
    //   try {
    //     Thread.sleep(1000);
    //     zeroHeading();
    //   } catch (Exception e) {
    //   }
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
    } else {
      speeds = new ChassisSpeeds(
        forwardSpeed,
        leftSpeed,
        rotationSpeed);
    }
    

    SwerveModuleState[] states =
      SwerveConstants.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public void zeroHeading() {
    navX.reset();
  }

  public double getHeading() {
    return navX.getAngle() % 360;
  }

  /** Gets the NavX angle as a Rotation2d. */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }
}
