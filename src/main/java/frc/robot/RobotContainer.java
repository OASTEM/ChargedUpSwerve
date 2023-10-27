

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.BalanceFront;
import frc.robot.commands.MoveTelescoping;
import frc.robot.commands.PadDrive;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;

import java.util.HashMap;
import java.util.function.Consumer;

import javax.management.InstanceAlreadyExistsException;
import javax.naming.OperationNotSupportedException;
import javax.net.ssl.SSLSocket;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;
//import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.ShuffleboardComponents;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Autos.BalanceAuto;
import frc.robot.Autos.BalanceAutoBackward;
import frc.robot.Autos.FullAuto;
import frc.robot.Autos.NoBalanceAuto;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ManipulatorCommands.ScoreCube;
import frc.robot.commands.ManipulatorCommands.ScoringPosition;
import frc.robot.commands.ManipulatorCommands.StopIntakeMotor;
import frc.robot.commands.ManipulatorCommands.IntakeCube;
import frc.robot.commands.ManipulatorCommands.IntakeGround;
import frc.robot.commands.ManipulatorCommands.MoveHigh;
import frc.robot.commands.ManipulatorCommands.MoveLow;
import frc.robot.commands.ManipulatorCommands.MoveMid;
import frc.robot.commands.ManipulatorCommands.Retract;
import frc.robot.commands.ManipulatorCommands.ScoreCone;

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
  private final Manipulator manipulator;
  private final LogitechGamingPad pad;
  private final Limelight limelight;
  private final LED led;
  private final ShuffleboardComponents components;
  private final JoystickButton padA;
  private final JoystickButton padB;
  private final JoystickButton padX;
  private final JoystickButton padY;
  private final JoystickButton rightBumper;
  private final JoystickButton leftBumper;
  
  private final LogitechGamingPad opPad;
  private final JoystickButton opPadA;
  private final JoystickButton opPadB;
  private final JoystickButton opPadX;
  private final JoystickButton opPadY;
  private final JoystickButton opRightBumper;
  private final JoystickButton opLeftBumper;

  private final PathPlannerTrajectory redPath;
  private final PathPlannerTrajectory bluePath;
  
  private final BalanceAuto balanceAuto;
  private final BalanceAutoBackward balanceAutoBackward;
  private final FullAuto fullAuto;
  private final NoBalanceAuto noBalanceAuto;

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // manipulator = new Manipulator();
    pad = new LogitechGamingPad(0);
    opPad = new LogitechGamingPad(1);
    limelight = new Limelight();
    led = new LED();
   
    
    padA = new JoystickButton(pad, 1);
    padB = new JoystickButton(pad, 2);
    padX = new JoystickButton(pad, 3);
    padY = new JoystickButton(pad, 4);
    rightBumper = new JoystickButton(pad, 6);
    leftBumper = new JoystickButton(pad, 5);

    opPadA = new JoystickButton(opPad, 1);
    opPadB = new JoystickButton(opPad, 2);
    opPadX = new JoystickButton(opPad, 3);
    opPadY = new JoystickButton(opPad, 4);
    opRightBumper = new JoystickButton(opPad, 6);
    opLeftBumper = new JoystickButton(opPad, 5);
  

    // NamedCommands.registerCommand("Auto Balance", new BalanceFront(swerveSubsystem));
    
    redPath = PathPlanner.loadPath("Straight Red Path", new PathConstraints(5, 4)); // in m/s\
    bluePath = PathPlanner.loadPath("Full Blue Path", new PathConstraints(2, 2)); // in m/s
    
    
    swerveSubsystem = new SwerveSubsystem(SwerveConstants.usingVision, limelight);
    manipulator = new Manipulator();
    components = new ShuffleboardComponents(swerveSubsystem, limelight);
    swerveSubsystem.setDefaultCommand(new PadDrive(swerveSubsystem, pad, true));
    manipulator.setDefaultCommand(new MoveTelescoping(manipulator, opPad));

    //Autos
    balanceAuto = new BalanceAuto(swerveSubsystem, manipulator);
    balanceAutoBackward = new BalanceAutoBackward(swerveSubsystem, manipulator);
    fullAuto = new FullAuto(swerveSubsystem, manipulator);
    noBalanceAuto = new NoBalanceAuto(swerveSubsystem, manipulator);

    //Configure auto chooser
    m_chooser.setDefaultOption("Balance Auto", balanceAuto);
    m_chooser.addOption("Balance Auto Backward", balanceAutoBackward);
    m_chooser.addOption("Full Auto", fullAuto);
    m_chooser.addOption("No Balance Auto", noBalanceAuto);
    SmartDashboard.putData("Auto Chooser", m_chooser);

    // Configure the button bindings
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
    padY.onTrue(new InstantCommand(swerveSubsystem::configAAcornMode));
    padX.onTrue(new BalanceFront(swerveSubsystem));
    rightBumper.whileTrue(new InstantCommand(manipulator::disableSoftLimit));
    leftBumper.onTrue(new InstantCommand(manipulator::enabelSoftLimit));

    // opPadA.whileTrue(new StopIntakeMotor(manipulator));
    // opPadB.whileTrue(new IntakeGround(manipulator));
    // opPadX.whileTrue(new ScoreCube(manipulator))
    
    // opPadY.whileTrue(new ScoringPosition(manipulator));
    opPadX.whileTrue(new Retract(manipulator)); 
    opPadA.whileTrue(new MoveHigh(manipulator));
    opPadB.whileTrue(new MoveMid(manipulator));
    opPadY.whileTrue(new MoveLow(manipulator));

    opRightBumper.whileTrue(new IntakeGround(manipulator));
    opRightBumper.whileFalse(new InstantCommand(manipulator::stopIntake));
    opLeftBumper.whileTrue(new ScoreCube(manipulator));

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return swerveSubsystem.followTrajectoryCommand(bluePath, true);

    return m_chooser.getSelected();
  }

  /**
    * Gets the test command
    *
    * @return the command to run in test initial
  //   */


  public void addRotorPositions(){
    swerveSubsystem.addRotorPositionsforModules();
  }
}