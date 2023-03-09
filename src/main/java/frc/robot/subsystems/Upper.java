package frc.robot.subsystems;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.PID;
import frc.ChenryLib.SettledUtility;
import frc.robot.Constants;

public class Upper extends SubsystemBase{
    private WPI_TalonFX elbowLeft;
    private WPI_TalonFX elbowRight;
    private CANCoder elbowEncoder;
    private SettledUtility elbowSU;

    private WPI_TalonFX stringboi;
    private CANCoder stringEncoder;
    private SettledUtility stringSU;


    private Solenoid squishyboi;
    

    private PID elbowPID;
    private PID stringPID;


    private double currentElbowTarget = 0;
    private double currentStringTarget = 0;

    boolean isIdle = true;

    boolean stringSettled = true;
    boolean elbowSettled = true;
    boolean rotationSettled = true;


    private static class elbowPos{
        static double mid = 0;
        static double high = 0;
        static double down = 0;
        static double human = 0;
    }

    private static class stringPos{
        static double high = 0;
        static double mid = 0;
        static double down = 0;
        static double human = 0;
    }

    boolean grabFromHuman = false;
    boolean scoreMid = false;
    boolean scoreHigh = false;
    boolean down = false;


    Upper(){
        elbowLeft = new WPI_TalonFX(Constants.Superstructure.talonLeftPort);
        elbowRight = new WPI_TalonFX(Constants.Superstructure.talonRightPort);
        elbowRight.follow(elbowLeft, FollowerType.PercentOutput);

        elbowEncoder = new CANCoder(Constants.Superstructure.elbowCanCoderPort);
        elbowEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        elbowEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        //can add invert

        elbowPID = new PID(0.001, 0, 0, 0, 0);
        
        stringboi = new WPI_TalonFX(Constants.Superstructure.talonStringPort);
        stringPID = new PID(0, 0, 0, 0, 0);


        squishyboi = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Superstructure.solenoidPort);
        

    }
    
    @Override
    public void periodic() {

        //fix overwriting issue with multiple being true;
        // if (grabFromHuman){
        //     setElbowTarget(elbowPos.human);
        //     elbowSettled = false;
        //     if (elbowSettled) {
        //         setStringTarget(stringPos.human);
        //         stringSettled = false;
        //     }
        //     setRotationTarget(rotationPos.horizontal);
        //     if (elbowSettled && stringSettled && rotationSettled) grabFromHuman = false;
        // }

        // if (down){
        //     setRotationTarget(rotationPos.standby);
        //     setStringTarget(stringPos.down);
        //     if (rotationSettled && stringSettled) {setElbowTarget(elbowPos.down); down = false;}
        // }

        // if (scoreMid){
        //     setElbowTarget(elbowPos.mid);
        //     elbowSettled = false;
        //     if (elbowSettled){
        //         setStringTarget(stringPos.mid);
        //         stringSettled = false;
        //     }
        //     if (stringSettled) {
        //         setRotationTarget(rotationPos.horizontal);
        //         scoreMid = false;
        //     }
        // }

        // if (scoreHigh){
        //     setElbowTarget(elbowPos.high);
        //     elbowSettled = false;
        //     if (elbowSettled){
        //         setStringTarget(stringPos.high);
        //         setRotationTarget(rotationPos.horizontal);
        //         scoreHigh = false;
        //     }
        // }





        double stringError = currentStringTarget - stringEncoder.getPosition();
        double elbowError = currentElbowTarget - elbowEncoder.getAbsolutePosition();
        updateStates(stringError, elbowError);
        SmartDashboard.putNumber("elbow abs pos", elbowEncoder.getAbsolutePosition());

        //rotationSet(rotationPID.calculate(rotationError));
        //stringSet(stringPID.calculate(stringError));
        //elbowSet(elbowPID.calculate(elbowError));
    }

    // void standbyIntakeSequence(){
    //     if (!isIdle) return;
    //     isIdle = false;
    //     boolean running = true;
    //     rotationSetTarget(rotationPos.standby);
    //     stringSetTarget(stringPos.standbyIntake);
    //     elbowSetTarget(elbowPos.standbyIntake);
    //     unClamp();
    //     running = false;
    //     if (!running) isIdle = true;
    // }

    // void scoreStandbySequence(){
        
    //     isIdle = false;
    //     rotationSetTarget(rotationPos.standby);
    //     stringSetTarget(stringPos.standbyIntake);
    //     if (stringSettled) elbowSetTarget(elbowPos.standbyIntake);
    //     if (stringSettled && elbowSettled) clamp();
    //     elbowSetTarget(elbowPos.standbyPut);
    //     isIdle = true;
    // }

    // void scoreSequence(){
    //     isIdle = false;
    //     elbowSetTarget(elbowPos.up);
    //     if (elbowSettled) stringSetTarget(stringPos.up);
    //     isIdle = true;
    // }

    void updateStates(double stringError, double elbowError){
        stringSettled = stringSU.isSettled(stringError);
        elbowSettled = elbowSU.isSettled(elbowError);
    }

    void elbowSet(double value){
        elbowLeft.set(ControlMode.PercentOutput, value);
    }

    void setElbowTarget(double target){
        currentElbowTarget = target;
    }

    void stringSet(double value){
        stringboi.set(ControlMode.PercentOutput, value);
    }

    void setStringTarget(double target){
        currentStringTarget = target;
    }

    void clamp (){
        squishyboi.set(true);
    }

    void unClamp(){
        squishyboi.set(false);
    }



}
