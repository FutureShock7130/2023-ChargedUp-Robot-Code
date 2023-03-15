package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;

public class turn extends CommandBase {
    Swerve drive;
    double rotateTarget;
    double currentRotation;
    double error;
    double settle = 0.1;//😱😱😱😱😱
    double output;
    double p, i, d;
    PID turnPID;
    

    public turn(Swerve swerve, double targetRotation, double kp, double ki, double kd){
        rotateTarget = targetRotation;
        drive = swerve;
        p = kp;
        i = ki;
        d = kd;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentRotation = drive.getYaw().getRotations();
    }

    @Override
    public void execute() {
        turnPID = new PID(p, i, d, rotateTarget, 1);//😱😱😱
        currentRotation = drive.getYaw().getRotations();
        output = turnPID.calculate(rotateTarget - currentRotation);
        // drive.drive(new Translation2d(0,0), turnPID.calculate(rotateTarget - drive.getYaw().getRotations()), false, false);
        
        SmartDashboard.putNumber("autoTurnYaw", currentRotation);
        SmartDashboard.putNumber("autoTurnOutput", output);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(rotateTarget - drive.getYaw().getRotations()) <= settle){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, false, true);
    }
}
