package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private Joystick controller;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private boolean fieldOriented = false;

  public TeleopSwerve(
      Swerve s_Swerve,
      Joystick iJoystick) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    controller = iJoystick;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(controller.getRawAxis(Constants.JoystickConstants.leftStick_Y), Constants.Swerve.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(controller.getRawAxis(Constants.JoystickConstants.leftStick_X), Constants.Swerve.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(controller.getRawAxis(Constants.JoystickConstants.rightStick_X), Constants.Swerve.stickDeadband));
    
    if (controller.getRawButtonPressed(Constants.JoystickConstants.btn_X)) fieldOriented = !fieldOriented;
    
    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity, fieldOriented,
        true);
  }
}
