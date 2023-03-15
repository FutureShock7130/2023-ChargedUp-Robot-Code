package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.PID;
import frc.ChenryLib.SettledUtility;
import frc.robot.subsystems.Swerve;

public class turn extends CommandBase {
    Swerve drive;
    double rotateTarget;
    double currentRotation;
    double error;
    double output;
    double p = 0.001;
    double i = 0;
    double d = 0;
    PID turnPID;
    SettledUtility settled;
    boolean finish;
    

    public turn(Swerve swerve, double targetDegrees){
        rotateTarget = targetDegrees;
        drive = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentRotation = drive.getYaw().getRotations();
    }

    @Override
    public void execute() {
        currentRotation = drive.getYaw().getRotations();
        error = Units.rotationsToDegrees(rotateTarget - currentRotation);
        turnPID = new PID(p, i, d, rotateTarget, 1);//😱😱😱
        settled = new SettledUtility(100, error, 10);
        output = turnPID.calculate(error);
        finish = settled.isSettled(error);
        
        drive.drive(new Translation2d(0,0), output, false, false);
        
        SmartDashboard.putNumber("autoTurnYaw", currentRotation);
        SmartDashboard.putNumber("autoTurnOutput", output);
        SmartDashboard.putNumber("autoTurnError",error);
    }

    @Override
    public boolean isFinished() {
        if(finish){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, false, true);
        drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0, 0)));
    }
}
