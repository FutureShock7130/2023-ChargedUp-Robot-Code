// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.vision.*;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Index.indexStates;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  // private finalXboxController driver = newXboxController(0);
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  UsbCamera intakeCamera = CameraServer.startAutomaticCapture();

  /* Drive Controls */
  // private final int translationAxis = XboxController.Axis.kLeftY.value;
  // private final int strafeAxis = XboxController.Axis.kLeftX.value;
  // private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  // private final JoystickButton zeroGyro =
  //     new JoystickButton(driver, XboxController.Button.kY.value);
  // private final JoystickButton robotCentric =
  //     new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Index index = new Index();
  private final ApriltagSubsystem apriltag = new ApriltagSubsystem();
  private final Upper upper = new Upper();
  //private final Upper Superstructure = new Upper();
  // private final Intake iIntake = new Intake();
  //private final TeleopSwerve driverSwerve = new TeleopSwerve(s_Swerve, -driver.getRawAxis(translationAxis), driver.getRawAxis(strafeAxis), driver.getRawAxis(rotationAxis), false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver));
    // index.setDefaultCommand(new TeleopIndex(index, driver));
    apriltag.setDefaultCommand(new RunCommand(()->{
      Boolean ok = fieldShoot.OKshoot(apriltag.getCameratoTarget());
      SmartDashboard.putBoolean("OKshoot", ok);
    }, apriltag));
    upper.setDefaultCommand(new TeleopUpper(upper, driver, operator, null));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.XboxXboxController} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    
    //in front view
    // y+ left 
    //x- front

    //blue left
    return new SequentialCommandGroup(

    new InstantCommand(()->{
      index.setState(indexStates.AimTop);
      Timer.delay(0.25);
    }),
    new ShootFor1s(index),

    new driveSide(s_Swerve, -0.3),//check
    new betterDelay(0.05),
    new drivefront(s_Swerve, 2.6896),
    new betterDelay(0.05),

    new turnForTime(s_Swerve, 1.25, 2.7),
    new betterDelay(0.05),

    new ParallelCommandGroup(
      new InstantCommand(()->{
        index.setState(indexStates.Indexing);
      }),
      new drivefront(s_Swerve, -3)
    ),
    new InstantCommand(()->{
      index.setState(indexStates.Standby);
      Timer.delay(0.25);
    }),
    new driveSide(s_Swerve, -1.5494),//right is positive (robot head is forward side)

    new turnForTime(s_Swerve, 1.25, 2.7),
    
    new drivefront(s_Swerve, -2.2098),
    new drivefront(s_Swerve, 1)


      // new indexAuto(index, indexStates.AimTop),
      // new betterDelay(0.5),


      // new InstantCommand(()->{
      //   index.shootByState();
      //   Timer.delay(1);
      // }),
      // new driveForTime(s_Swerve, 4, -1),
      // new turnForTime(s_Swerve, 1.25, 2.7), // 180
      // new sideForTime(s_Swerve, 2, 1)
    );
    // return new SequentialCommandGroup(
    //   new indexAuto(index, indexStates.AimTop),
    //   new InstantCommand(()->{
    //     index.shootByState();
    //   }),
    //   new turn(s_Swerve, 180),
    //   new drivefront(s_Swerve, -5.6896),
    //   new indexAuto(index, indexStates.Indexing),
    //   new ParallelCommandGroup(
    //   new indexAuto(index, indexStates.Standby),
    //   new driveSide(s_Swerve, -1.2192)
    //   ),
    //   new drivefront(s_Swerve, 0.78105),
    //   new balance(s_Swerve)
    // );

  }
}
