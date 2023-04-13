// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

public class SwerveModule {
  private TalonFX driveMotor;
  private TalonFX steerMotor;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveId, int steerId) {
    driveMotor = new TalonFX(driveId);
    steerMotor = new TalonFX(steerId);

    driveMotor.configFactoryDefault();
    steerMotor.configFactoryDefault();

    driveMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setDriveSpeed(double speed) {
    driveMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setSteerSpeed(double speed) {
    steerMotor.set(ControlMode.PercentOutput, speed);
  }

  private void setSteerPosition(double degrees) {
  }

  public double encoderToAngle(double encoderCount) {
    return encoderCount * 360 / MotorConstants.encoderCountsPerRotation * MotorConstants.angleMotorGearRatio;
  }

  public double angleToEncoder(double angle) {
    return angle * MotorConstants.encoderCountsPerRotation / 360 / MotorConstants.angleMotorGearRatio;
  }

  public void drive(SwerveModuleState state) {
    // setDriveSpeed(state.speedMetersPerSecond);
    // setSteerSpeed(state.angle.getDegrees());
    state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(encoderToAngle(steerMotor.getSelectedSensorPosition())));
    double stateAngle = state.angle.getDegrees();

    setSteerPosition(angleToEncoder(state.angle.getDegrees()));
    setDriveSpeed(state.speedMetersPerSecond / MotorConstants.maxSpeed);
  }
}
