package frc.robot.commands;

import java.lang.invoke.ConstantBootstraps;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.MathUtility;
import frc.robot.Constants;
import frc.robot.Constants.intake;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends CommandBase {
  private Intake intake;
  private Joystick controller;
  private Joystick operator;

  private int currentTilterPosCounter = 3;
  public TeleopIntake(Intake iIntake, Joystick iJoystick, Joystick iJoystick2) {
    this.intake = iIntake;
    addRequirements(iIntake);
    controller = iJoystick;
    operator = iJoystick2;
  }

  @Override
  public void execute() {
    // if (controller.getRawButton(Constants.JoystickConstants.btn_RS)) intakeToggle = !intakeToggle;
    // if (controller.getRawButton(Constants.JoystickConstants.btn_RB)) {
    //   intakeToggle = false;
    //   intake.setRollers(-0.8);
    // }
    // if (intakeToggle) intake.setRollers(1);
    




    // if (controller.getRawButton(Constants.JoystickConstants.btn_LB)){
    //   currentTilterPosCounter++;
    //   currentTilterPosCounter = (int) MathUtility.clamp(currentTilterPosCounter, 0, 3);
    //   intake.setTilterPos(currentTilterPosCounter);
    // }
    // if (controller.getRawButton(Constants.JoystickConstants.btn_LS)){
    //   currentTilterPosCounter--;
    //   currentTilterPosCounter = (int) MathUtility.clamp(currentTilterPosCounter, 0, 3);
    //   intake.setTilterPos(currentTilterPosCounter);
    // }
    // if (controller.getRawButton(Constants.JoystickConstants.btn_A)){
    //   intake.setTilterPos(3);
    // }
    // if (controller.getRawButtonPressed(Constants.JoystickConstants.btn_RB)) intake.shoot();


        if (operator.getRawButton(Constants.JoystickConstants.btn_Y)) intake.setRollers(0.3);
        else if (operator.getRawButton(Constants.JoystickConstants.btn_A)) intake.setRollers(-0.6);
        else if (operator.getRawButton(Constants.JoystickConstants.btn_B)) intake.setRollers(-0.35);
        else if (operator.getRawButton(Constants.JoystickConstants.btn_X)) intake.setRollers(-1);
        else intake.setRollers(0);

        if (operator.getPOV() == 90) intake.clamp();
        if (operator.getPOV() == 270) intake.unClamp();
        if (operator.getRawAxis(Constants.JoystickConstants.trigger_R) > 0.6) intake.clamp();
        if (operator.getRawAxis(Constants.JoystickConstants.trigger_L) > 0.6) intake.unClamp();
        intake.tilterSet(operator.getRawAxis(Constants.JoystickConstants.rightStick_Y) * 0.3);
        //SmartDashboard.putNumber("lr axis", operator.getRawAxis(Constants.JoystickConstants.trigger_L));
        //SmartDashboard.putNumber("rtrigger axis", operator.getRawAxis(Constants.JoystickConstants.trigger_R));
    //timing + mutex and then we turn off the intake, and when the intake is down we automaitcally intake
}
}