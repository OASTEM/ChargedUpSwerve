// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.utils.PID;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private CANSparkMax intakeMotor;
  private CANSparkMax pivotMotor;
  private CANSparkMax telescopingMotor;

  private SparkMaxPIDController pivotPIDController;
  private RelativeEncoder pivotEncoder;
  
  private SparkMaxPIDController telescopingPIDController;
  private RelativeEncoder telescopingEncoder;

  private DigitalInput coneSensor;
  private DigitalInput cubeSensor;

  public Manipulator() {
    intakeMotor = new CANSparkMax(ManipulatorConstants.INTAKE_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    pivotMotor = new CANSparkMax(ManipulatorConstants.PIVOT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    telescopingMotor = new CANSparkMax(ManipulatorConstants.TELESCOPING_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

    pivotPIDController = pivotMotor.getPIDController();
    pivotEncoder = pivotMotor.getEncoder();

    telescopingPIDController = telescopingMotor.getPIDController();
    telescopingEncoder = telescopingMotor.getEncoder();

    pivotPIDController.setP(ManipulatorConstants.pivotPID.p);
    pivotPIDController.setI(ManipulatorConstants.pivotPID.i);
    pivotPIDController.setD(ManipulatorConstants.pivotPID.d);

    telescopingPIDController.setP(ManipulatorConstants.telescopingPID.p);
    telescopingPIDController.setI(ManipulatorConstants.telescopingPID.i);
    telescopingPIDController.setD(ManipulatorConstants.telescopingPID.d);

    cubeSensor = new DigitalInput(ManipulatorConstants.INTAKE_SENSOR_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Intake Functions
  public void spinIntake() {
    intakeMotor.set(ManipulatorConstants.INTAKE_SPEED);
  }
  
  public void reverseIntake() {
    intakeMotor.set(ManipulatorConstants.REVERSE_INTAKE_SPEED);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  // Pivot Functions

  public void setPivotPosition(double position){
    pivotEncoder.setPosition(position);
  }

  public void initPivotPIDController(PID pid){
    pivotPIDController.setP(pid.p);
    pivotPIDController.setI(pid.i);
    pivotPIDController.setD(pid.d);
  }


  // Telescoping Functions
  
  public void setTelescopingPosition(double position){
    telescopingEncoder.setPosition(position);
  }

  public void initTelescopingPIDController(PID pid){
    telescopingPIDController.setP(pid.p);
    telescopingPIDController.setI(pid.i);
    telescopingPIDController.setD(pid.d);
  }

  // Extension Functions

  public void extendGround(){
    setPivotPosition(ManipulatorConstants.PIVOT_GROUND_EXTENDED_POSITION);
    setTelescopingPosition(ManipulatorConstants.TELESCOPING_EXTENDED_POSITION);
    spinIntake();
  }

  public void extendLow(){
    setPivotPosition(ManipulatorConstants.PIVOT_LOW_POSITION);
    setTelescopingPosition(ManipulatorConstants.TELESCOPING_EXTENDED_POSITION);
  }

  public void extendHigh(){
    setPivotPosition(ManipulatorConstants.PIVOT_HIGH_POSITION);
    setTelescopingPosition((ManipulatorConstants.TELESCOPING_EXTENDED_POSITION));
  }

  public void extendMid(){
    setPivotPosition(ManipulatorConstants.PIVOT_MID_POSITION);
    setTelescopingPosition(ManipulatorConstants.TELESCOPING_EXTENDED_POSITION);
  }

  public void intakeFeederStation(){
    setPivotPosition(ManipulatorConstants.PIVOT_FEEDER_STATION);
    setTelescopingPosition(ManipulatorConstants.TELESCOPING_EXTENDED_POSITION);
    reverseIntake();
  }


  // Retraction Functions

  public void retract(){
    stopIntake();
    setPivotPosition(ManipulatorConstants.PIVOT_RETRACTED_POSITION);
    setTelescopingPosition(ManipulatorConstants.TELESCOPING_RETRACTED_POSITION);
  }

  // sensor functions

  public boolean getCubeSensor(){
    return cubeSensor.get();
  }

  public boolean getConeSensor(){
    return coneSensor.get();
  }

  // Current functions

  public double getPivotCurrent(){
    return pivotMotor.getAppliedOutput();
  }

  public double getTeleArmCurrent(){
    return telescopingMotor.getOutputCurrent();
  }
}
