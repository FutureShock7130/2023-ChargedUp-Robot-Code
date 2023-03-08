package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.MathUtility;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends CommandBase {
  private Intake intake;
  private Joystick controller;

  private boolean intakeToggle = false;
  private int currentTilterPosCounter = 3;
  public TeleopIntake(
      Intake iIntake,
      Joystick iJoystick) {
    this.intake = iIntake;
    addRequirements(iIntake);
    controller = iJoystick;
  }

  @Override
  public void execute() {
    if (controller.getRawButton(Constants.JoystickConstants.btn_RS)) intakeToggle = !intakeToggle;
    if (controller.getRawButton(Constants.JoystickConstants.btn_RB)) {
      intakeToggle = false;
      intake.setRollers(-0.8);
    }
    if (intakeToggle) intake.setRollers(1);
    
    
    
    if (controller.getRawButton(Constants.JoystickConstants.btn_LB)){
      currentTilterPosCounter++;
      currentTilterPosCounter = (int) MathUtility.clamp(currentTilterPosCounter, 0, 3);
      intake.setTilterPos(currentTilterPosCounter);
    }
    if (controller.getRawButton(Constants.JoystickConstants.btn_LS)){
      currentTilterPosCounter--;
      currentTilterPosCounter = (int) MathUtility.clamp(currentTilterPosCounter, 0, 3);
      intake.setTilterPos(currentTilterPosCounter);
    }
}
}