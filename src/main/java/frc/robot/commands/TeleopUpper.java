package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Upper;
import frc.robot.subsystems.Upper.States;

public class TeleopUpper extends CommandBase {
  private Upper superstructure;
  private Joystick controller;

  public TeleopUpper(Upper iSuperstructure, Joystick iJoystick) {
    this.superstructure = iSuperstructure;
    addRequirements(iSuperstructure);
    this.controller = iJoystick;
  }

  @Override
  public void execute() {
    if (controller.getRawButton(Constants.JoystickConstants.btn_Y)) superstructure.setStates(States.coneHigh);
    if (controller.getRawButton(Constants.JoystickConstants.btn_A)) superstructure.setStates(States.down);
  }
}