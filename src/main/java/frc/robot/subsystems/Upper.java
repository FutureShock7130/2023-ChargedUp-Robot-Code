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

public class Upper extends SubsystemBase {
    public static Object states;
    private WPI_TalonFX elbowLeft = new WPI_TalonFX(Constants.Superstructure.talonLeftPort);
    private WPI_TalonFX elbowRight = new WPI_TalonFX(Constants.Superstructure.talonRightPort);;
    private CANCoder elbowEncoder = new CANCoder(Constants.Superstructure.elbowCanCoderPort);
    private SettledUtility elbowSU = new SettledUtility(100, 5, 2);

    private WPI_TalonFX stringboi = new WPI_TalonFX(Constants.Superstructure.talonStringPort);
    private CANCoder stringEncoder = new CANCoder(Constants.Superstructure.stringCanCoderPort);
    private SettledUtility stringSU = new SettledUtility(100, 50, 50);

    private Solenoid squishyboi = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Superstructure.solenoidPort);

    private PID elbowPID = new PID(0, 0, 0, 0, 0);
    private PID stringPID = new PID(0, 0, 0, 0, 0);

    private double currentElbowTarget = elbowPos.down;
    private double currentStringTarget = stringPos.down;

    boolean stringSettled = true;
    boolean elbowSettled = true;

    boolean elbowIsOutside = false;

    private static class elbowPos {
        static double mid = 0;
        static double high = 0;
        static double down = 0;
        static double human = 0;
        static double placingOffset = 0;
        static double outside = 0;
    }

    private static class stringPos {
        static double high = 0;
        static double mid = 0;
        static double down = 0;
        static double human = 0;
    }

    public static enum States {
        placing,
        human,
        coneHigh,
        coneMid,
        down,
    }

    States state = States.down;
    States lastState = States.down;

    public Upper() {
        elbowRight.follow(elbowLeft, FollowerType.PercentOutput);

        elbowEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        elbowEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        // can add invert

        stringEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    @Override
    public void periodic() {
        double stringError = currentStringTarget - stringEncoder.getPosition();
        double elbowError = currentElbowTarget - elbowEncoder.getAbsolutePosition();
        
        switch (state){
            case down:
                setStringTarget(stringPos.down);
                stringError = currentStringTarget - stringEncoder.getPosition();
                elbowError = currentElbowTarget - elbowEncoder.getAbsolutePosition();
                updateStates(stringError, elbowError);
                if (stringSettled) setElbowTarget(elbowPos.down);;
                break;
            case coneHigh:
                setElbowTarget(elbowPos.high);
                updateStates(stringError, elbowError);
                if (elbowIsOutside) setStringTarget(stringPos.high);
                break;
            case coneMid:
                setElbowTarget(elbowPos.mid);
                updateStates(stringError, elbowError);
                if (elbowIsOutside) setStringTarget(stringPos.mid);
                break;
            case human:
                setElbowTarget(elbowPos.human);
                updateStates(stringError, elbowError);
                if (elbowIsOutside) setStringTarget(stringPos.human);
                break;
            case placing:
                setElbowTarget(currentElbowTarget + elbowPos.placingOffset);
                break;
        }

        lastState = state;


        updateStates(stringError, elbowError);
        SmartDashboard.putNumber("elbow abs pos", elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("string enc pos", stringEncoder.getPosition());

        stringSet(stringPID.calculate(stringError));
        elbowSet(elbowPID.calculate(elbowError));
    }

    public void setStates (States istate){
        state = istate;
    }

    void updateStates(double stringError, double elbowError) {
        stringSettled = stringSU.isSettled(stringError);
        elbowSettled = elbowSU.isSettled(elbowError);
        if (elbowEncoder.getAbsolutePosition() > elbowPos.outside)
            elbowIsOutside = true;
        else
            elbowIsOutside = false;
    }

    void elbowSet(double value) {
        //elbowLeft.set(ControlMode.PercentOutput, value);
    }

    void setElbowTarget(double target) {
        currentElbowTarget = target;
    }

    void stringSet(double value) {
        //stringboi.set(ControlMode.PercentOutput, value);
    }

    void setStringTarget(double target) {
        currentStringTarget = target;
    }

    void clamp() {
        squishyboi.set(true);
    }

    void unClamp() {
        squishyboi.set(false);
    }
}
