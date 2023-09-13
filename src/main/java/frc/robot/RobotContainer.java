// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.BalanceFront;
import frc.robot.commands.PadDrive;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.StartIntake;
//import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Manipulator.Intake;
import frc.robot.utils.LogitechGamingPad;

import java.util.HashMap;
import java.util.function.Consumer;

import javax.net.ssl.SSLSocket;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.ShuffleboardComponents;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final Intake intake;
  private final LogitechGamingPad pad;
  private final Limelight limelight;
  private final ShuffleboardComponents components;
  
  private final JoystickButton padA;
  private final JoystickButton padB;
  private final JoystickButton padX;
  private final JoystickButton padY;
  private final JoystickButton rightBumper;
  private final JoystickButton leftBumper;
  
  private final PathPlannerTrajectory examplePath;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveSubsystem = Robot.swerveSubsystem;
    intake = new Intake();
    pad = new LogitechGamingPad(0);
    limelight = new Limelight();
    components = new ShuffleboardComponents();
    
    padA = new JoystickButton(pad, 1);
    padB = new JoystickButton(pad, 2);
    padX = new JoystickButton(pad, 3);
    padY = new JoystickButton(pad, 4);
    rightBumper = new JoystickButton(pad, 6);
    leftBumper = new JoystickButton(pad, 5);
    
    examplePath = PathPlanner.loadPath("Test Path", new PathConstraints(4, 3)); // in m/s
    
    swerveSubsystem.setDefaultCommand(
        new PadDrive(
            swerveSubsystem, pad, true, limelight, Constants.SwerveConstants.usingVision));
    
    configureBindings();
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    padA.onTrue(new InstantCommand(swerveSubsystem::addRotorPositionsforModules));
    padB.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    padY.onTrue(new InstantCommand(swerveSubsystem::configSlowMode));
    padX.whileTrue(new BalanceFront(swerveSubsystem));
    
    leftBumper.whileTrue(new StartIntake(intake));
    rightBumper.whileTrue(new ReverseIntake(intake));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Consumer<SwerveModuleState[]> outputModuleStates = moduleStates -> {
      for (int i = 0; i < moduleStates.length; i++) {
          SwerveModuleState moduleState = moduleStates[i];
          // System.out.println("Module " + i + " - Angle: " + moduleState.angle + ", Speed: " + moduleState.speedMetersPerSecond);
          System.out.println("Test 2 ***********************************");
      }
    };
    return swerveSubsystem.followTrajectoryCommand(examplePath, false);
  }
}


