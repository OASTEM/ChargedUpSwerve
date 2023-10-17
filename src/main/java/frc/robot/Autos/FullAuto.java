// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balance;
import frc.robot.commands.BalanceFront;
import frc.robot.commands.DriveStraight;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullAuto extends SequentialCommandGroup {
  /** Creates a new AutoBalance. */
  private SwerveSubsystem swerveSubsystem;

  public FullAuto(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(swerveSubsystem::zeroHeading),
      new InstantCommand(swerveSubsystem::addRotorPositionsforModules),
      new DriveStraight(swerveSubsystem, 1).withTimeout(0.5),
      new DriveStraight(swerveSubsystem, -1).withTimeout(0.5),
      new DriveStraight(swerveSubsystem, 1).withTimeout(5),
      new DriveStraight(swerveSubsystem, -1).withTimeout(3),
      new BalanceFront(swerveSubsystem)
    );
  }
}
