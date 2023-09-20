// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
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
import frc.robot.Constants.DebugMode.DebugPIDS;

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
  private double initialPos;

  private final double CANCoderDriveStraightSteerSetPoint;

  private CurrentLimitsConfigs driveCurrentLimitsConfigs;
  private CurrentLimitsConfigs steerCurrentLimitsConfigs;

  private ClosedLoopRampsConfigs driveClosedRampsConfigs;
  private ClosedLoopRampsConfigs steerClosedRampsConfigs;

  static int printSlower = 0;

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

    driveslot0Configs.kP = SwerveConstants.PIDConstants.DRIVE_PID.p;
    driveslot0Configs.kI = SwerveConstants.PIDConstants.DRIVE_PID.i;
    driveslot0Configs.kD = SwerveConstants.PIDConstants.DRIVE_PID.d;

    steerslot0Configs.kP = SwerveConstants.PIDConstants.STEER_PID.p; //original 0.06
    steerslot0Configs.kI = SwerveConstants.PIDConstants.STEER_PID.i;
    steerslot0Configs.kD = SwerveConstants.PIDConstants.STEER_PID.d; //Original 0.008

    driveMotor.getConfigurator().apply(driveslot0Configs);
    steerMotor.getConfigurator().apply(steerslot0Configs);

    // initialCANCoderValue = canCoder.getAbsolutePosition().refresh().getValue();
    // m_cycle.Position = (initialCANCoderValue - CANCoderDriveStraightSteerSetPoint) * Constants.MotorConstants.STEER_MOTOR_GEAR_RATIO;
    // steerMotor.setControl(m_position.withPosition((initialCANCoderValue - CANCoderDriveStraightSteerSetPoint) * Constants.MotorConstants.STEER_MOTOR_GEAR_RATIO));

    steerConfigurator.setRotorPosition(CANCoderDriveStraightSteerSetPoint);
        // steerMotor.setRotorPosition(-(initialCANCoderValue - CANCoderDriveStraightSteerSetPoint) * Constants.MotorConstants.STEER_MOTOR_GEAR_RATIO);

    // canCoder.setPositionToAbsolute();

    driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
    steerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    driveClosedRampsConfigs = new ClosedLoopRampsConfigs();
    steerClosedRampsConfigs = new ClosedLoopRampsConfigs();

    // currentLimitsConfigs.SupplyCurrentLimit = 40;
    // currentLimitsConfigs.StatorCurrentLimit = 40;
    // currentLimitsConfigs.Enable = true;
    
    driveClosedRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.1;
    steerClosedRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.05;

    driveMotor.getConfigurator().apply(driveCurrentLimitsConfigs);
    steerMotor.getConfigurator().apply(steerCurrentLimitsConfigs);

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
    // System.out.println("Rotor Pos " + steerMotor.getRotorPosition());
    // System.out.println("Setting Steer " + rotations);
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
    state = optimize(state,
        Rotation2d.fromDegrees(
            rotationsToAngle(steerMotor.getRotorPosition().getValue(),
                MotorConstants.STEER_MOTOR_GEAR_RATIO)));
    
    var currentRotations = (steerMotor.getRotorPosition().getValue());

    var currentAngle = Rotation2d.fromDegrees(
        rotationsToAngle(currentRotations, MotorConstants.STEER_MOTOR_GEAR_RATIO));     
        

    // System.out.println(state.speedMetersPerSecond);
    setDriveSpeed(state.speedMetersPerSecond / MotorConstants.MAX_SPEED);

    // if (Math.abs(state.speedMetersPerSecond) < SwerveConstants.STATE_SPEED_THRESHOLD){
    //   System.out.println("Relevant state number g");
    // }
    

    if (Math.abs(state.speedMetersPerSecond) > SwerveConstants.STATE_SPEED_THRESHOLD) {
      double newRotations;
      // SmartDashboard.putNumber("Set Falcon " + this.steerMotor.getDeviceID(), newRotations);
      var delta = state.angle.minus(currentAngle);

      double change = delta.getDegrees();

      if (change > 90){
        change -= 180;
      }

      else if (change < -90){
        change += 180;
      }

      newRotations = currentRotations + angleToRotations(change, MotorConstants.STEER_MOTOR_GEAR_RATIO);
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

  public void setRotorPos(){
    initialCANCoderValue = canCoder.getAbsolutePosition().refresh().getValue();
    steerMotor.setRotorPosition(-(initialCANCoderValue - CANCoderDriveStraightSteerSetPoint) * Constants.MotorConstants.STEER_MOTOR_GEAR_RATIO);
  }

  public static SwerveModuleState optimize(

    
      SwerveModuleState desiredState, Rotation2d currentAngle) {

    // System.out.println(currentAngle.minus(desiredState.angle).getDegrees());   

    var currentDegrees = currentAngle.getDegrees();
    var desiredDegrees = desiredState.angle.getDegrees();

    SmartDashboard.putNumber("Difference: " , currentAngle.minus(desiredState.angle).getDegrees()); 
    while (currentDegrees - desiredDegrees > 180.0){
      desiredDegrees = desiredDegrees + 360;
    }

    while (currentDegrees - desiredDegrees < -180.0){
      desiredDegrees = desiredDegrees - 360;
    }



    // if (printSlower == 100) {
    //    System.out.println("Desired: " + desiredState.angle.getDegrees());
    //    System.out.println("Current: " + currentAngle.getDegrees());
    //    System.out.println("Difference: " + currentAngle.minus(desiredState.angle).getDegrees());   

    //   printSlower = 0;
    
    // }

    // else{
    //     printSlower++;
    // }
    var delta = desiredState.angle.minus(currentAngle);
    
    // if (delta.getDegrees() > 90) {
    //  delta = delta.minus(Rotation2d.fromDegrees(180));
    // }

    // else if (delta.getDegrees() < -90){
    //   delta = delta.plus(Rotation2d.fromDegrees(180));
    // }

    
    if (delta.getDegrees() > 90.0) {

      return new SwerveModuleState(

          
          -desiredState.speedMetersPerSecond,
          Rotation2d.fromDegrees(desiredDegrees - 180));
    
    }

    else if (delta.getDegrees() < - 90) {
      return new SwerveModuleState(

      -desiredState.speedMetersPerSecond,
      Rotation2d.fromDegrees(desiredDegrees + 180));
      
      
     } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d.fromDegrees(desiredDegrees));
    }
  }

    public void setDebugPID (boolean debugPID){
      if (debugPID){
        steerslot0Configs.kP = DebugPIDS.debugSteer.p;
        steerslot0Configs.kI = DebugPIDS.debugSteer.i;
        steerslot0Configs.kD = DebugPIDS.debugSteer.d;
  
        driveslot0Configs.kP = DebugPIDS.debugDrive.p;
        driveslot0Configs.kI = DebugPIDS.debugDrive.i;
        driveslot0Configs.kD = DebugPIDS.debugDrive.d;
      }
      else{
        driveslot0Configs.kP = SwerveConstants.PIDConstants.DRIVE_PID.p;
        driveslot0Configs.kI = SwerveConstants.PIDConstants.DRIVE_PID.i;
        driveslot0Configs.kD = SwerveConstants.PIDConstants.DRIVE_PID.d;

        steerslot0Configs.kP = SwerveConstants.PIDConstants.STEER_PID.p; //original 0.06
        steerslot0Configs.kI = SwerveConstants.PIDConstants.STEER_PID.i;
        steerslot0Configs.kD = SwerveConstants.PIDConstants.STEER_PID.d;
      }
    }

}
