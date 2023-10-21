// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.utils.LogitechGamingPad;

public class MoveTelescoping extends CommandBase {
  /** Creates a new MoveTelescoping. */
  private Manipulator manipulator;
  private LogitechGamingPad pad;
  public MoveTelescoping(Manipulator manipulator, LogitechGamingPad pad) {
    addRequirements(manipulator);
    this.manipulator = manipulator;
    this.pad = pad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    manipulator.setTelescopingSpeed(4 * pad.getLeftAnalogYAxis());

    if (Math.abs(-pad.getRightAnalogYAxis()) > 0.05 )
    {
    manipulator.setPivotSpeed(-pad.getRightAnalogYAxis());
    }
    else
    {
      manipulator.setPivotSpeed(0);
    }
    // else
    // {
    //   manipulator.setPivotSpeed(0);
    //   manipulator.holdPivot();
    // }

    // if (!manipulator.getConeSensor())
    // {
    //   manipulator.holdCone();
    // }

    if (Math.abs(pad.getLeftTriggerValue()) > 0.3){
      manipulator.coneScore();
      SmartDashboard.putBoolean("intake stopped", false);
    }

    if (Math.abs(pad.getRightTriggerValue()) > 0.3){
      manipulator.coneIntake();
    }

    if (Math.abs(pad.getRightTriggerValue()) < 0.3 && Math.abs(pad.getLeftTriggerValue()) < 0.3){
      manipulator.stopIntake();
      SmartDashboard.putBoolean("intake stopped", true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulator.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
