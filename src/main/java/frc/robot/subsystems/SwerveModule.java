// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.motorcontrol.ControlMode;
import com.ctre.phoenix6.motorcontrol.NeutralMode;
import com.ctre.phoenix6.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveConstants;

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

    steerMotor.config_kP(0, 0);
    steerMotor.config_kI(0, 0);
    steerMotor.config_kD(0, 0);

    resetEncoders();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        encoderToMeters(
            driveMotor.getSelectedSensorPosition(), MotorConstants.DRIVE_GEAR_RATIO
        ),
        Rotation2d.fromDegrees(
            encoderToAngle(steerMotor.getSelectedSensorPosition(),
                           MotorConstants.STEER_GEAR_RATIO)
        )
    );
  }

  public void setDriveSpeed(double speed) {
    driveMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setSteerSpeed(double speed) {
    steerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setSteerPosition(double encoderCount) {
    steerMotor.set(ControlMode.Position, encoderCount);
  }

  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    steerMotor.setSelectedSensorPosition(0);
  }

  /** Converts encoder counts to degrees. */
  public double encoderToAngle(double encoderCount, double gearRatio) {
    return encoderCount * 360 /
        MotorConstants.ENCODER_COUNTS_PER_ROTATION * gearRatio;
  }

  /** Converts degrees to encoder counts. */
  public double angleToEncoder(double angle, double gearRatio) {
    return angle * MotorConstants.ENCODER_COUNTS_PER_ROTATION / 360 /
        gearRatio;
  }

  public double encoderToMeters(double encoderCount, double gearRatio) {
    return encoderCount / (MotorConstants.ENCODER_COUNTS_PER_ROTATION *
        gearRatio) * MotorConstants.WHEEL_DIAMETER * Math.PI;
  }

  public double metersToEncoder(double meters, double gearRatio) {
    return meters / (MotorConstants.WHEEL_DIAMETER * Math.PI) *
        MotorConstants.ENCODER_COUNTS_PER_ROTATION * gearRatio;
  }

  public void setState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state,
        Rotation2d.fromDegrees(
            encoderToAngle(steerMotor.getSelectedSensorPosition(),
                MotorConstants.STEER_MOTOR_GEAR_RATIO)));

    setDriveSpeed(state.speedMetersPerSecond / MotorConstants.MAX_SPEED);

    if (Math.abs(state.speedMetersPerSecond) > SwerveConstants.STATE_SPEED_THRESHOLD) {
      setSteerPosition(angleToEncoder(
        state.angle.getDegrees(), MotorConstants.STEER_MOTOR_GEAR_RATIO));
    }
  }

  public void stop() {
    setDriveSpeed(0);
    setSteerSpeed(0);
  }
}
