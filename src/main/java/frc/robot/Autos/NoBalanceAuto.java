// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NoBalanceAuto extends SequentialCommandGroup {
  /** Creates a new AutoBalance. */

  public NoBalanceAuto(SwerveSubsystem swerveSubsystem, Manipulator manipulator) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(swerveSubsystem::addRotorPositionsforModules),
      new DriveStraight(swerveSubsystem, 0.9, manipulator).withTimeout(4.5)
    );
  }
}