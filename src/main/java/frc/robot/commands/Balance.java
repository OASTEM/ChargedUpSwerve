// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.PID;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Balance extends CommandBase {
  // private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);

  /** Creates a new balance. */
  SwerveSubsystem driveTrain;
  private double error;                                                                 
  private final double goal = 0;
  private final double maxEffort = 1;
  private double p, i, d;
  //PID Values need to be tuned
  //Might need to create two pid values for both sides of the drivetrain
  //PID balancePID = new PID(0.023, 0.002, 0.002);
  PID balancePID;

  public Balance(SwerveSubsystem driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    p = SmartDashboard.getNumber("P", 0.015);
    i = SmartDashboard.getNumber("I", 0.00015);
    d = SmartDashboard.getNumber("D", 0.0008);
    SmartDashboard.putNumber("P", p);
    SmartDashboard.putNumber("I", i);
    SmartDashboard.putNumber("D", d);
                                                                              
    balancePID = new PID(p, i, d, 0);
    driveTrain.stop();
    driveTrain.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Error is equal to the NavX getYaw (using getRoll)
    // driveTrain.setLeftSpeed(0.3);
    // driveTrain.setBackLeftSpeed();
    // driveTrain.setRightSpeed(0.3);
    error = driveTrain.getRoll() + 4.5;

    double effort = balancePID.calculate(goal, error);
    if (effort < -maxEffort) {
      effort = -maxEffort;
    } else if (effort > maxEffort) {
      effort = maxEffort;
    }

    // pitch = driveTrain.getPitch();

    //     for(int i = 0; i<3; i++){
    //         if(pitch < 0){
    //             new DriveAmount(driveTrain, distance, speed, true);
    //         }
    //     }

    double speed = effort * -5;
    driveTrain.drive(speed, 0, 0, true);


    SmartDashboard.putNumber("navXYError", this.error);
    SmartDashboard.putNumber("PID Speed", effort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    p = SmartDashboard.getNumber("P", 0.021);
    i = SmartDashboard.getNumber("I", 0.002);
    d = SmartDashboard.getNumber("D", 0.002);
    balancePID = new PID(p, i, d,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


