// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Index.indexStates;

public class ShootFor1s extends CommandBase {

  Index index;

  /** Creates a new ShootFor1s. */
  public ShootFor1s(Index index) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.index = index;
    addRequirements(index);
  }

  private double time;
  private double lastTime;
  private double dt;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    index.shootByState();
    lastTime = Timer.getFPGATimestamp();
    dt = lastTime - time;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.setRollers(0);
    index.setState(indexStates.Standby);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (dt >= 1) return true;
    else return false;
  }
}
