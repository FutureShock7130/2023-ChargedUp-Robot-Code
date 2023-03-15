package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;

public class drivefront extends CommandBase{
    Swerve drive;
    double targetDis;
    double currentDistance;
    double p, i, d, output;
    PID frontPID;
    double settle = 0.1;//😱😱😱😱

    public drivefront(Swerve swerve, double targetDis, double kp, double ki, double kd){
        drive = swerve;
        this.targetDis = targetDis;
        p = kp;
        i = ki;
        d = kd;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        frontPID = new PID(p, i, d, targetDis, 1);//😱😱😱😱😱
    }

    @Override
    public void execute() {
        currentDistance = drive.getPose().getX();
        output = frontPID.calculate(targetDis - currentDistance);
        // drive.drive(new Translation2d(output, 0), 0, false, true);

        SmartDashboard.putNumber("autoFrontX", currentDistance);
        SmartDashboard.putNumber("autoFrontOutput", output);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(currentDistance - targetDis) <= settle){
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
