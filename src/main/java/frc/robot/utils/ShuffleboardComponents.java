// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BalanceDebug;
import frc.robot.Constants.SwerveConstants;

public class ShuffleboardComponents extends SubsystemBase {
  /** Creates a new Shuffleboard. */
  // All shuffleboard tabs
  private ShuffleboardTab vision;
  private ShuffleboardTab driver;
  private ShuffleboardTab debug;
  private ShuffleboardTab swerve;
  private ShuffleboardTab prematch;

  private SimpleWidget entry;

  public ShuffleboardComponents() {
    vision = Shuffleboard.getTab("Vision");
    driver = Shuffleboard.getTab("Driver");
    debug = Shuffleboard.getTab("Debug");
    swerve = Shuffleboard.getTab("Swerve");
    prematch = Shuffleboard.getTab("Prematch");

    //Create components
    entry = vision.add("Using Vision", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Vision Methods
    entry.getEntry().setBoolean(SwerveConstants.usingVision);
    //Driver Methods
    // driver.add("Slow Mode On", true);
    //Debug Methods
    // BalanceDebug.P = debug.add("Balance P", 0).getEntry().get().getDouble();
    // BalanceDebug.I = debug.add("Balance I", 0).getEntry().get().getDouble();
    // BalanceDebug.D = debug.add("Balance D", 0).getEntry().get().getDouble();
    //Swerve Methods

    //Prematch Methods

    System.out.println(SwerveConstants.usingVision);
  }
}
