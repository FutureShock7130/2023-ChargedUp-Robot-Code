// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Index.indexStates;

public class TeleopIndex extends CommandBase {

  private Index index;
  // private Joystick jDriver;
  private XboxController xDriver;
  private Joystick jOperator;
  
  /** Creates a new TeleopIndex. */
  public TeleopIndex(Index index, XboxController Driver, Joystick jOperator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.index = index;
    this.xDriver = Driver;
    this.jOperator = jOperator;
    addRequirements(index);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Auto -> Driver
    // Manual -> Operator

    // Autos
    if (xDriver.getRightTriggerAxis() > 0.6 && xDriver.getYButton()) index.setState(indexStates.AimTopCS, true);
    if (xDriver.getRightTriggerAxis() > 0.6 && xDriver.getBButton()) index.setState(indexStates.AimMiddleCS, true);
    if (xDriver.getRightTriggerAxis() > 0.6 && xDriver.getAButton()) index.setState(indexStates.AimBottomCS, true);
    if (xDriver.getYButton()) index.setState(indexStates.AimTop, true);
    if (xDriver.getBButton()) index.setState(indexStates.AimMiddle, true);
    if (xDriver.getAButton()) index.setState(indexStates.AimBottom, true);
    if (xDriver.getLeftTriggerAxis() > 0.6) index.setState(indexStates.Indexing, false);
    if (xDriver.getLeftBumper()) index.setState(indexStates.Standby, false);

    // Manuals
    if (jOperator.getPOV() == 180) index.toggleClamp(); // clamp & unclamp
    if (jOperator.getRawButton(JoystickConstants.btn_LB)) index.setRollers(-0.28); // intake
    if (jOperator.getRawAxis(JoystickConstants.trigger_L) > 0.4) index.setRollers(1); // shoot
    if (jOperator.getRawAxis(JoystickConstants.leftStick_Y) > 0.28) index.setTilterPosBySpd(jOperator.getRawAxis(JoystickConstants.leftStick_Y) * 0.3); // tilter
  }

}
