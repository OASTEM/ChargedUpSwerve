// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DebugMode;
import frc.robot.Constants.DebugMode.DebugPIDS;
import frc.robot.Constants.SwerveConstants.PIDConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.PID;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Balance extends CommandBase {
  // private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);

  /** Creates a new balance. */
  SwerveSubsystem swerve;
  private double error;                                                                 
  private final double goal = 0;
  private final double maxEffort = 1;
  private double p, i, d;
  //PID Values need to be tuned
  //Might need to create two pid values for both sides of the swerve
  //PID balancePID = new PID(0.023, 0.002, 0.002);
  PID balancePID;

  public Balance(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    balancePID = PIDConstants.BALANCE_PID;
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    swerve.zeroHeading();

  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Error is equal to the NavX getYaw (using getRoll)
    // swerve.setLeftSpeed(0.3);
    // swerve.setBackLeftSpeed();
    // swerve.setRightSpeed(0.3);
    swerve.drive(1, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    // balancePID = new PID(p, i, d,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

