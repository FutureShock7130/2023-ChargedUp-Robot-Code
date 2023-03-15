package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  // private Joystick controller;
  private XboxController xJoystick;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private boolean fieldOriented = false;

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  public TeleopSwerve(
      Swerve s_Swerve,
      XboxController xController) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.xJoystick = xController;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    if (xJoystick.getRightBumper()) {
      translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(xJoystick.getLeftY() * 0.4, Constants.Swerve.stickDeadband));
      strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(xJoystick.getLeftX() * 0.4, Constants.Swerve.stickDeadband));
      rotationVal =
          rotationLimiter.calculate(
              MathUtil.applyDeadband(xJoystick.getRightX() * 0.2, Constants.Swerve.stickDeadband));
    } else {
      translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(xJoystick.getLeftY(), Constants.Swerve.stickDeadband));
      strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(xJoystick.getLeftX(), Constants.Swerve.stickDeadband));
      rotationVal =
          rotationLimiter.calculate(
              MathUtil.applyDeadband(xJoystick.getRightX() * 0.5, Constants.Swerve.stickDeadband));
    }
    
    if (xJoystick.getStartButton()) fieldOriented = !fieldOriented;  
    if (xJoystick.getBackButton()) s_Swerve.zeroGyro();

    int thing = fieldOriented ? 1 : 0;
    SmartDashboard.putNumber("fo ", thing);
    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity, fieldOriented,
        true);
  }
}
