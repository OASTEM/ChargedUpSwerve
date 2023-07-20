// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private TalonFX driveMotor;
  private TalonFX steerMotor;
  private CANcoder canCoder;

  private MotorOutputConfigs motorConfigs;

  private TalonFXConfigurator driveConfigurator;
  private TalonFXConfigurator steerConfigurator;

  private Slot0Configs driveslot0Configs;
  private Slot0Configs steerslot0Configs;

  private DutyCycleOut m_request;

  private PositionVoltage m_position;
  private PositionDutyCycle m_cycle;
  private double initialCANCoderValue;

  private final double CANCoderDriveStraightSteerSetPoint;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveId, int steerId, int canCoderID, double CANCoderDriveStraightSteerSetPoint) {
    driveMotor = new TalonFX(driveId);
    steerMotor = new TalonFX(steerId);
    canCoder = new CANcoder(canCoderID);

    this.CANCoderDriveStraightSteerSetPoint = CANCoderDriveStraightSteerSetPoint;

    motorConfigs = new MotorOutputConfigs();

    driveConfigurator = driveMotor.getConfigurator();
    steerConfigurator = steerMotor.getConfigurator();

    driveslot0Configs = new Slot0Configs();
    steerslot0Configs = new Slot0Configs();
    
    m_request = new DutyCycleOut(0);
    m_position = new PositionVoltage(0);
    m_cycle = new PositionDutyCycle(0);

    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    steerMotor.getConfigurator().apply(new TalonFXConfiguration());

    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    driveConfigurator.apply(motorConfigs);
    steerConfigurator.apply(motorConfigs);

    driveslot0Configs.kP = 0.01;
    driveslot0Configs.kI = 0;
    driveslot0Configs.kD = 0;

    steerslot0Configs.kP = 0.1;
    steerslot0Configs.kI = 0;
    steerslot0Configs.kD = 0;
    
    resetEncoders();

    driveMotor.getConfigurator().apply(driveslot0Configs);
    steerMotor.getConfigurator().apply(steerslot0Configs);

    initialCANCoderValue = canCoder.getAbsolutePosition().refresh().getValue();
    m_cycle.Position = (initialCANCoderValue - CANCoderDriveStraightSteerSetPoint) * Constants.MotorConstants.STEER_MOTOR_GEAR_RATIO;
    // steerMotor.setControl(m_position.withPosition((initialCANCoderValue - CANCoderDriveStraightSteerSetPoint) * Constants.MotorConstants.STEER_MOTOR_GEAR_RATIO));

    // canCoder.setPositionToAbsolute();

  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        encoderToMeters(
            driveMotor.getRotorPosition().getValue(), MotorConstants.DRIVE_MOTOR_GEAR_RATIO
        ),
        Rotation2d.fromDegrees(
            encoderToAngle(steerMotor.getRotorPosition().getValue(),
                           MotorConstants.STEER_MOTOR_GEAR_RATIO)
        )
    );
  }

  public void setDriveSpeed(double speed) {
    driveMotor.setControl(m_request.withOutput(speed));
  }

  public void setSteerSpeed(double speed) {
    steerMotor.setControl(m_request.withOutput(speed));
  }

  public void setSteerPosition(double rotations) {
    System.out.println("Rotor Pos " + steerMotor.getRotorPosition());
    System.out.println("Setting Steer " + rotations);
    steerMotor.setControl(m_cycle.withPosition(rotations));
    //steerMotor.setRotorPosition(rotations);
  }

  public void resetEncoders() {
    driveMotor.setRotorPosition(0);
    //steerMotor.setRotorPosition(0);
  }

  /** Converts encoder counts to degrees. */
  public double encoderToAngle(double encoderCount, double gearRatio) {
    return encoderCount * 360 /
        MotorConstants.ENCODER_COUNTS_PER_ROTATION * gearRatio;
  }

  public double rotationsToAngle(double rotations, double gearRatio){
    return rotations * 360 / gearRatio;
  }

  public double angleToRotations(double angle, double gearRatio) {
    return angle / 360 * gearRatio;
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
            rotationsToAngle(steerMotor.getRotorPosition().getValue(),
                MotorConstants.STEER_MOTOR_GEAR_RATIO)));

    // System.out.println(state.speedMetersPerSecond);
    setDriveSpeed(state.speedMetersPerSecond / MotorConstants.MAX_SPEED);

    if (Math.abs(state.speedMetersPerSecond) > SwerveConstants.STATE_SPEED_THRESHOLD) {
      double newRotations = angleToRotations(
        state.angle.getDegrees(), MotorConstants.STEER_MOTOR_GEAR_RATIO);
      SmartDashboard.putNumber("Set Falcon " + this.steerMotor.getDeviceID(), newRotations);
      setSteerPosition(newRotations);
    }
  }

  public void stop() {
    setDriveSpeed(0);
    setSteerSpeed(0);
  }

  public void updateSteerPositionSmartDashboard() {
    SmartDashboard.putNumber("Actual Falcon " + this.steerMotor.getDeviceID(), this.steerMotor.getRotorPosition().refresh().getValue());
    SmartDashboard.putNumber("CAN Coder Value" + canCoder.getDeviceID(), canCoder.getAbsolutePosition().getValue());
  }

  public void setSteerTo0() {
    steerMotor.setRotorPosition(0);
  }
}
