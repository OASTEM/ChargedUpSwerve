// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private TalonFX driveMotor;
  private TalonFX steerMotor;
  private CANCoder canCoder;

  private MotorOutputConfigs motorConfigs;

  private TalonFXConfigurator driveConfigurator;
  private TalonFXConfigurator steerConfigurator;

  private Slot0Configs driveslot0Configs;
  private Slot0Configs steerslot0Configs;

  private DutyCycleOut m_request;

  private PositionVoltage m_position;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveId, int steerId, int canCoderID) {
    driveMotor = new TalonFX(driveId);
    steerMotor = new TalonFX(steerId);
    canCoder = new CANCoder(canCoderID);

    motorConfigs = new MotorOutputConfigs();

    driveConfigurator = driveMotor.getConfigurator();
    steerConfigurator = steerMotor.getConfigurator();

    driveslot0Configs = new Slot0Configs();
    steerslot0Configs = new Slot0Configs();
    
    m_request = new DutyCycleOut(0);
    m_position = new PositionVoltage(0);

    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    steerMotor.getConfigurator().apply(new TalonFXConfiguration());

    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    driveConfigurator.apply(motorConfigs);
    steerConfigurator.apply(motorConfigs);

    driveslot0Configs.kP = 0;
    driveslot0Configs.kI = 0;
    driveslot0Configs.kD = 0;

    steerslot0Configs.kP = 0;
    steerslot0Configs.kI = 0;
    steerslot0Configs.kD = 0;

    resetEncoders();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        encoderToMeters(
            driveMotor.getRotorPosition().getValue(), MotorConstants.DRIVE_GEAR_RATIO
        ),
        Rotation2d.fromDegrees(
            encoderToAngle(steerMotor.getRotorPosition().getValue(),
                           MotorConstants.STEER_GEAR_RATIO)
        )
    );
  }

  public void setDriveSpeed(double speed) {
    driveMotor.setControl(m_request.withOutput(speed));
  }

  public void setSteerSpeed(double speed) {
    steerMotor.setControl(m_request.withOutput(speed));
  }

  public void setSteerPosition(double encoderCount) {
    steerMotor.setControl(m_position.withPosition(encoderCount));
  }

  public void resetEncoders() {
    driveMotor.setRotorPosition(0);
    steerMotor.setRotorPosition(0);
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
            encoderToAngle(steerMotor.getRotorPosition().getValue(),
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
