package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;;

public class balance extends CommandBase{
    Swerve drive;
    double output;
    double balanceSpeed = 0.1;
    double currentPitch;
    PID balancePID;
    double p = 0.0001;
    double i = 0;
    double d = 0;
    double settle = 1;//degrees
    double aboard = 5;
    double define;

    public balance(Swerve swerve){
        drive = swerve;
        addRequirements(swerve);
    }
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        currentPitch = drive.getPitch();
        define = currentPitch > 0 ? 1:-1;
        if(Math.abs(currentPitch) >= aboard){
            drive.drive(new Translation2d(define*balanceSpeed, 0), 0, false, false);
        }

    }

    @Override
    public boolean isFinished() {
      if(Math.abs(currentPitch) <= settle){
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
