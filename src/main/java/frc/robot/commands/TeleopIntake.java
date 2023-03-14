package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.RollerStates;
import frc.robot.subsystems.Intake.States;

public class TeleopIntake extends CommandBase {
  private Intake intake;
  private Joystick controller;


  public TeleopIntake(Intake iIntake, Joystick iJoystick) {
    this.intake = iIntake;
    addRequirements(iIntake);
    controller = iJoystick;

  }

  @Override
  public void execute() {
    //human, puts the intake up, opens the clamp and intakes
    if (controller.getRawButton(Constants.JoystickConstants.btn_A)) intake.setStates(States.human, RollerStates.intake);
    //puts the intake down, opens the clamp and intakes
    if (controller.getRawButton(Constants.JoystickConstants.btn_A)) intake.setStates(States.down, RollerStates.intake);
    //closes the clamp and moves the intake to mid
    if (controller.getRawButton(Constants.JoystickConstants.btn_A)) intake.setStates(States.mid, RollerStates.intake);
    //closes the clamp and moves the intake to mid if its not already in mid and then shoots high, will instantly shoot if its already in mid 
    if (controller.getRawButton(Constants.JoystickConstants.btn_A)) intake.setStates(States.mid, RollerStates.shootHigh);
    //closes the clamp and moves the intake to mid if its not already in mid and then shoots mid, will instantly shoot if its already in mid 
    if (controller.getRawButton(Constants.JoystickConstants.btn_A)) intake.setStates(States.mid, RollerStates.shootMid);
    //will shoot at full speed regardless of the position
    if (controller.getRawButton(Constants.JoystickConstants.btn_A)) intake.setStates(intake.getState(), RollerStates.shootFull);
}
}