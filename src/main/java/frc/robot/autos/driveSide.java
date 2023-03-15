package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;

public class driveSide extends CommandBase{
    Swerve drive;
    double targetDis;
    double currentDistance;
    double p, i, d, output;
    PID sidePID;
    double settle = 0.001;//😱😱😱😱

    public driveSide(Swerve swerve, double targetDis, double kp, double ki, double kd){
        drive = swerve;
        this.targetDis = targetDis;
        p = kp;
        i = ki;
        d = kd;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        sidePID = new PID(p, i, d, targetDis, 1);//😱😱😱😱😱
    }

    @Override
    public void execute() {
        currentDistance = drive.getPose().getY();
        output = sidePID.calculate(targetDis - currentDistance);
        // drive.drive(new Translation2d(0, output), 0, false, true);

        SmartDashboard.putNumber("autoSideY", currentDistance);
        SmartDashboard.putNumber("autoSideOutput", output);
        SmartDashboard.putNumber("autoSideError", targetDis - currentDistance);
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
        drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0, 0)));
    }


}
