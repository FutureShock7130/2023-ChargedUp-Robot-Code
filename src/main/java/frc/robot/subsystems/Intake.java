package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private CANSparkMax leftIntake;
    private CANSparkMax rightIntake;
    private Solenoid intakeClamp;
    private WPI_TalonFX tilter;
    private PID tilterPID;
    private double currentTilterTarget;

    static class tilterPos{
        public static double up = 90;
        public static double down = 0;
        public static double third = 45;
        public static double second = 30;
    }

    public Intake(){
        leftIntake = new CANSparkMax(Constants.intake.leftMotorPort, MotorType.kBrushless);
        rightIntake = new CANSparkMax(Constants.intake.rightMotorPort, MotorType.kBrushless);
        rightIntake.setInverted(true);
        intakeClamp = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.intake.solenoidPort);
        tilter = new WPI_TalonFX(Constants.intake.tilterPort);
        tilterPID = new PID(0.0001, 0, 0, 0, 0);
    }

    @Override
    public void periodic() {

        
        double tilterError = currentTilterTarget - tilter.getSelectedSensorPosition();
        tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterError));
    }

    public void setTilterTarget(double target){
        currentTilterTarget = target;
    }

    public void setTilterPos(int posIndex){
        posIndex = (int) MathUtility.clamp(posIndex, 0, 3);
        switch (posIndex){
            case 0: {
                setTilterTarget(tilterPos.down);
            }
            case 1: {
                setTilterTarget(tilterPos.second);
            }
            case 2: {
                setTilterTarget(tilterPos.third);
            }
            case 3: {
                setTilterTarget(tilterPos.up);
            }
        }
    }

    public void setRollers(double speed){
        leftIntake.set(speed);
        rightIntake.set(speed);
    }

    public void clamp(){
        intakeClamp.set(true);
    }

    public void unClamp(){
        intakeClamp.set(false);
    }
    
}
