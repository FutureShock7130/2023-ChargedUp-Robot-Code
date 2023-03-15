package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.ChenryLib.SetPointPID;
import frc.robot.subsystems.Swerve;;

public class balance extends CommandBase{
    Swerve drive;
    double output;
    double balanceSpeed = 0.1;
    double currentPitch;
    double last, error;
    SetPointPID balancePID;
    double p = 14;
    double i = 0;
    double d = 0;
    double settle = 0.5;//degrees
    double aboard = 1.5;
    double define;

    public balance(Swerve swerve){
        drive = swerve;
        addRequirements(swerve);
    }
    @Override
    public void initialize() {
        last = 0;
    }

    @Override
    public void execute() {
        currentPitch = drive.getRoll();
        define = currentPitch > 0 ? 1:-1;

        error = last - currentPitch;

        balancePID = new SetPointPID(p, i, d, 0, 1);
        output =MathUtility.clamp(balancePID.calculate(error), -0.5, 0.5);
        if(Math.abs(currentPitch) >= aboard){
            drive.drive(new Translation2d(define*Math.abs(output), 0), 0, false, false);
        }

        SmartDashboard.putNumber("PITCH", currentPitch);
        SmartDashboard.putNumber("rollError", error);
        SmartDashboard.putNumber("balanceOutput", output);

        last = currentPitch;


    }

    @Override
    public boolean isFinished() {
      if(Math.abs(error) <= settle){
        Timer.delay(3);
    
        return true;
      }else{
        return false;
      }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, false, false);
    }
}
