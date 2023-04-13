// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  private SwerveModule frontLeft;
  private SwerveModule frontRight;
  private SwerveModule backLeft;
  private SwerveModule backRight;

  private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    frontLeft = new SwerveModule(Constants.MotorConstants.frontLeftDriveId, Constants.MotorConstants.frontLeftSteerId);
    frontRight = new SwerveModule(Constants.MotorConstants.frontRightDriveId, Constants.MotorConstants.frontRightSteerId);
    backLeft = new SwerveModule(Constants.MotorConstants.backLeftDriveId, Constants.MotorConstants.backLeftSteerId);
    backRight = new SwerveModule(Constants.MotorConstants.backRightDriveId, Constants.MotorConstants.backRightSteerId);
  }

  public void drive(double forwardSpeed, double leftSpeed, double rotationSpeed) {
    ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, leftSpeed, rotationSpeed);
    SwerveModuleState[] states = Constants.SwerveConstants.kinematics.toSwerveModuleStates(speeds);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
