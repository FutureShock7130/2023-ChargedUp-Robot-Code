// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Swerve;

// public class LockWheel extends CommandBase {

//   private Swerve mSwerve;

//   /** Creates a new LockWheel. */
//   public LockWheel(Swerve mSwerve) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.mSwerve = mSwerve;
//     addRequirements(mSwerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     Rotation2d rot0 = new Rotation2d(Math.toRadians(115));
//     Rotation2d rot1 = new Rotation2d(Math.toRadians(45));
//     Rotation2d rot2 = new Rotation2d(Math.toRadians(115));
//     Rotation2d rot3 = new Rotation2d(Math.toRadians(45));

//     SwerveModuleState state0 = new SwerveModuleState(0, rot0);
//     SwerveModuleState state1 = new SwerveModuleState(0, rot1);
//     SwerveModuleState state2 = new SwerveModuleState(0, rot2);
//     SwerveModuleState state3 = new SwerveModuleState(0, rot3);

//     SwerveModuleState[] states = {state0, state1, state2, state3};

//     mSwerve.setModuleStates(states);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
