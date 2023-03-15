package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.SetPointPID;
import frc.robot.subsystems.Swerve;;

public class balance extends CommandBase{
    Swerve drive;
    double output;
    double balanceSpeed = 0.1;
    double currentFrontPitch, currentBackPitch;
    double last, error;
    SetPointPID balancePID;
    double p = 5;
    double i = 0;
    double d = 0;
    double settle = 1.5;//degrees
    double define;

    public balance(Swerve swerve){
        drive = swerve;
        addRequirements(swerve);
    }
    @Override
    public void initialize() {
        last = drive.getFrontRoll();
    }

    @Override
    public void execute() {
        currentFrontPitch = drive.getFrontRoll();
        error = currentFrontPitch - last;
        define = currentFrontPitch > 0 ? 1:-1;
        balancePID = new SetPointPID(p, i, d, 0, 1);
        output =MathUtility.clamp(balancePID.calculate(error), -0.5, 0.5);

        if(Math.abs(error) > 10){
            drive.drive(new Translation2d(output, 0), 0, false, true);
        }else{
            drive.drive(new Translation2d(define*0.01, 0), 0, false, true);
        }
       

        


        SmartDashboard.putNumber("PITCH", currentFrontPitch);
        SmartDashboard.putNumber("rollError", error);
        SmartDashboard.putNumber("balanceOutput", output);

        last = currentFrontPitch;


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
