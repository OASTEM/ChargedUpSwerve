// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator.Intake;
import frc.robot.subsystems.Manipulator.Pivot;
import frc.robot.subsystems.Manipulator.TelescopingArm;

public class IntakeBottom extends CommandBase {
  private final Intake intake;
  private final Pivot pivot;
  private final TelescopingArm telescopingArm;

  /** Creates a new IntakeBottom. */
  public IntakeBottom() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = new Intake();
    pivot = new Pivot();
    telescopingArm = new TelescopingArm();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setPosition(0.0);
    telescopingArm.setSpeed(0.5);
    intake.spinIntake(0.5);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
