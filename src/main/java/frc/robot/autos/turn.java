package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ChenryLib.PID;
import frc.robot.subsystems.Swerve;

public class turn extends CommandBase {
    Swerve drive;
    double rotateTarget;
    double currentRotation;
    double error;
    double settle = 1;//😱😱😱😱😱
    

    public turn(Swerve swerve, double rotation){
        rotateTarget = rotation;
        drive = swerve;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        currentRotation = drive.getYaw().getRotations();
    }

    @Override
    public void execute() {
        PID turnPID = new PID(0.00001, 0, 0, rotateTarget, 1);//😱😱😱
        drive.drive(new Translation2d(0,0), turnPID.calculate(rotateTarget - drive.getYaw().getRotations()), false, false);
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
