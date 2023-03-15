package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;;

public class balance extends CommandBase{
    Swerve drive;
    double output;
    double balance = 0;
    double currentPitch;
    PID balancePID;
    double p = 0.0001;
    double i = 0;
    double d = 0;
    double settle = 0.01;

    public balance(Swerve swerve){
        drive = swerve;
        addRequirements(swerve);
    }
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        balancePID = new PID(p, i, d, balance, 1);
        currentPitch = drive.getPitch();
        output = balancePID.calculate(balance - currentPitch);

        drive.drive(new Translation2d(output, 0), 0, false, true);

        SmartDashboard.putNumber("balanceError", balance - currentPitch);
        SmartDashboard.putNumber("balanceOutput", output);
        SmartDashboard.putNumber("balancePitch", currentPitch);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(balance - currentPitch) <= settle){
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
