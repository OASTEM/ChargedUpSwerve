// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ToggleLimeLight;
import frc.robot.utils.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  final NetworkTable m_limelightTable;
  private LimelightHelpers m_helpers = new LimelightHelpers();
  LimelightHelpers.LimelightResults llresults;
  double tv, tx, ta;
  private Pose2d robotPose_FieldSpace;
  
  public Limelight() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    llresults = LimelightHelpers.getLatestResults("");
  }


  

  @Override
  public void periodic() {
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
    // This method will be called once per scheduler run
    tv = m_limelightTable.getEntry("tv").getDouble(0);
    tx = m_limelightTable.getEntry("tx").getDouble(0);
    ta = m_limelightTable.getEntry("ta").getDouble(0);

    SmartDashboard.getBoolean("On Red", true);
    robotPose_FieldSpace = llresults.targetingResults.getBotPose2d_wpiRed();
    SmartDashboard.putNumber("ta", ta);
    SmartDashboard.putData("Toggle Lime Light", new ToggleLimeLight(this));
  }

  public double getTx(){
    return tx;
  }

  public double getTa(){
    return ta;
  }

  public double getTv(){
    return tv;
  }

  public void setLEDMode(double mode){
    m_limelightTable.getEntry("ledMode").setNumber(mode);
  }

  public void setPipeline(double pipeline){
    m_limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void setCamMode(double mode){
    m_limelightTable.getEntry("camMode").setNumber(mode);
  }

  public void setSnapshot(double mode){
    m_limelightTable.getEntry("snapshot").setNumber(mode);
  }

  public void setStream(double mode){
    m_limelightTable.getEntry("stream").setNumber(mode);
  }

  public void setAdvanced(double mode){
    m_limelightTable.getEntry("advanced_mode").setNumber(mode);
  }

  public Pose2d getRobotPosition(){
    return robotPose_FieldSpace;
  }
}
