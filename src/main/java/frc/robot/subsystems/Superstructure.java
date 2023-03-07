package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.PID;
import frc.ChenryLib.SettledUtility;
import frc.robot.Constants;

public class Superstructure extends SubsystemBase{
    private WPI_TalonFX elbowLeft;
    private WPI_TalonFX elbowRight;
    private CANCoder elbowEncoder;
    private SettledUtility elbowSU;

    private WPI_TalonFX stringboi;
    private CANCoder stringEncoder;
    private SettledUtility stringSU;

    private CANSparkMax rotationboi;
    private RelativeEncoder rotationEncoder;
    private SettledUtility rotationSU;

    private Solenoid squishyboi;
    

    private PID elbowPID;
    private PID stringPID;
    private PID rotationPID;


    private double currentElbowTarget = 0;
    private double currentStringTarget = 0;
    private double currentRotationTarget = 0;

    private static class rotationPos {
        static double standby = 0;
        static double horizontal = 0;
        static double flippedHorizontal = 0;
    }

    private static class elbowPos{
        static double mid = 0;
        static double up = 0;
        static double down = 0;
        static double standbyIntake = 0;
        static double standbyPut = 0;
    }

    private static class stringPos{
        static double up = 0;
        static double mid = 0;
        static double down = 0;
        static double standbyIntake = 0;
    }


    enum armStates{
        intakeStandby,
        scoreStandby,

    }

    Superstructure(){
        elbowLeft = new WPI_TalonFX(Constants.Superstructure.talonLeftPort);
        elbowRight = new WPI_TalonFX(Constants.Superstructure.talonRightPort);
        elbowRight.follow(elbowLeft, FollowerType.PercentOutput);
        elbowEncoder = new CANCoder(Constants.Superstructure.elbowCanCoderPort);
        elbowPID = new PID(1, 0, 0, 0, 0);

        stringboi = new WPI_TalonFX(Constants.Superstructure.talonStringPort);
        rotationboi = new CANSparkMax(Constants.Superstructure.neoRotationPort, MotorType.kBrushless);
        rotationEncoder = rotationboi.getEncoder();

        squishyboi = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Superstructure.solenoidPort);
    }
    
    @Override
    public void periodic() {

        
        
        double rotationError = currentRotationTarget - rotationEncoder.getPosition();
        rotationSet(rotationPID.calculate(rotationError));
        
        double stringError = currentStringTarget - stringEncoder.getPosition();
        stringSet(stringPID.calculate(stringError));
        
        double elbowError = currentElbowTarget - elbowEncoder.getAbsolutePosition();
        elbowSet(elbowPID.calculate(elbowError));
    }

    void elbowSet(double value){
        elbowLeft.set(ControlMode.PercentOutput, value);
    }

    void elbowSetTarget(double target){
        currentElbowTarget = target;
    }

    void stringSet(double value){
        stringboi.set(ControlMode.PercentOutput, value);
    }

    void stringSetTarget(double target){
        currentStringTarget = target;
    }

    void rotationSet (double value){
        rotationboi.set(value);
    }

    void rotationSetTarget (double target){
        currentRotationTarget = target;
    }



}
