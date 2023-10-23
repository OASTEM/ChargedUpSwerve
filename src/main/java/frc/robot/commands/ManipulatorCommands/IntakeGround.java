// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.Manipulator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeGround extends SequentialCommandGroup {
  /** Creates a new IntakeGround. */
  public IntakeGround(Manipulator manipulator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MovePivot(manipulator, ManipulatorConstants.PIVOT_GROUND_INTAKE_POSITION).withTimeout(2),
      new MoveTele(manipulator, ManipulatorConstants.TELESCOPING_GROUND_INTAKE_POSITION).withTimeout(2),
      new MovePivot(manipulator, ManipulatorConstants.PIVOT_GROUND_INTAKE_POSITION2).withTimeout(1),
      new IntakeCube(manipulator)
    );
  }
}