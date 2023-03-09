package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
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

    boolean stringSettled = true;
    boolean elbowSettled = true;


    private static class elbowPos{
        static double mid = 0;
        static double high = 0;
        static double down = 0;
        static double human = 0;
        static double placingOffset = 0;
    }

    private static class stringPos{
        static double high = 0;
        static double mid = 0;
        static double down = 0;
        static double human = 0;
    }

    enum States {
        placing,
        human,
        coneHigh,
        coneMid,
        standby,
    }


    States state = States.standby;


    Upper(){
        elbowLeft = new WPI_TalonFX(Constants.Superstructure.talonLeftPort);
        elbowRight = new WPI_TalonFX(Constants.Superstructure.talonRightPort);
        elbowRight.follow(elbowLeft, FollowerType.PercentOutput);

        elbowEncoder = new CANCoder(Constants.Superstructure.elbowCanCoderPort);
        elbowEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        elbowEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        //can add invert

        elbowPID = new PID(0.0001, 0, 0, 0, 0);
        
        stringboi = new WPI_TalonFX(Constants.Superstructure.talonStringPort);
        stringPID = new PID(0, 0, 0, 0, 0);


        squishyboi = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Superstructure.solenoidPort);
    }
    
    @Override
    public void periodic() {
        double stringError = currentStringTarget - stringEncoder.getPosition();
        double elbowError = currentElbowTarget - elbowEncoder.getAbsolutePosition();
        
        switch (state){
            case standby:
                setStringTarget(stringPos.down);
                if (stringSettled) setElbowTarget(elbowPos.down);;
                break;
            case coneHigh:
                setElbowTarget(elbowPos.high);
                if (elbowSettled) setStringTarget(stringPos.high);
                break;
            case coneMid:
                setElbowTarget(elbowPos.mid);
                if (elbowSettled) setStringTarget(stringPos.mid);
                break;
            case human:
                setElbowTarget(elbowPos.human);
                setStringTarget(stringPos.human);
                break;
            case placing:
                setElbowTarget(currentElbowTarget + elbowPos.placingOffset);
                break;
        }


        updateStates(stringError, elbowError);
        SmartDashboard.putNumber("elbow abs pos", elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("string enc pos", stringEncoder.getPosition());

        //stringSet(stringPID.calculate(stringError));
        //elbowSet(elbowPID.calculate(elbowError));
    }


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
