package frc.robot.commands.ManipulatorCommands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class ExtendMid extends CommandBase {
  /** Creates a new IntakeGround. */
  private Manipulator manipulator;

  public ExtendMid(Manipulator manipulator) {
    addRequirements(manipulator);
    this.manipulator = manipulator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipulator.telescopingArmExtend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //   manipulator.extendMid();
    manipulator.setPivotPosition(Constants.ManipulatorConstants.PIVOT_MID_POSITION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
