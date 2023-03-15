// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Index.indexStates;

public class TeleopIndex extends CommandBase {

  private Index index;
  // private Joystick jDriver;
  private XboxController xDriver;
  
  /** Creates a new TeleopIndex. */
  public TeleopIndex(Index index, XboxController Driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.index = index;
    this.xDriver = Driver;
    addRequirements(index);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Autos
    if (xDriver.getYButton()) index.setState(indexStates.AimTop);
    if (xDriver.getBButton()) index.setState(indexStates.AimMiddle);
    if (xDriver.getAButton()) index.setState(indexStates.AimBottom);
    if (xDriver.getXButton()) index.shootByState();
    if (xDriver.getLeftTriggerAxis() > 0.6) index.setState(indexStates.Indexing);
    if (xDriver.getLeftBumper()) index.setState(indexStates.IndexHuman);
    if (xDriver.getRightTriggerAxis() > 0.6) index.setState(indexStates.Standby);
  }

}
