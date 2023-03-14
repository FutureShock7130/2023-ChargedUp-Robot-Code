package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.ChenryLib.SettledUtility;
import frc.robot.Constants;

public class Upper extends SubsystemBase {

    public static Object states;

    // Elbow
    private WPI_TalonFX elbowLeft = new WPI_TalonFX(Constants.Superstructure.talonLeftPort, "7130");
    private WPI_TalonFX elbowRight = new WPI_TalonFX(Constants.Superstructure.talonRightPort, "7130");;
    private CANCoder elbowEncoder = new CANCoder(Constants.Superstructure.elbowCanCoderPort, "7130");
    private SettledUtility elbowSU = new SettledUtility(100, 5, 2);

    // String
    private WPI_TalonFX stringboi = new WPI_TalonFX(Constants.Superstructure.talonStringPort, "7130");
    private CANCoder stringEncoder = new CANCoder(Constants.Superstructure.stringCanCoderPort, "7130");
    private SettledUtility stringSU = new SettledUtility(100, 50, 50);

    // Grabber
    // private Solenoid squishyboi = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Superstructure.solenoidPort);
    private CANSparkMax grabberLeft = new CANSparkMax(Constants.Superstructure.grabberLeft, MotorType.kBrushless);
    private CANSparkMax grabberRight = new CANSparkMax(Constants.Superstructure.grabberRight, MotorType.kBrushless);

    private PID elbowPID = new PID(0.04, 0, 0.01, 0, 0);
    private PID stringPID = new PID(0.01, 0, 0, 0, 0);

    private double currentElbowTarget = elbowPos.down;
    private double currentStringTarget = stringPos.down;

    boolean stringSettled = true;
    boolean elbowSettled = true;

    boolean elbowIsOutside = false;
    boolean stringIsInside = true;

    private static class elbowPos {
        static double mid = 71;
        static double high = 57;
        static double down = 155;
        static double human = 60; //untested 🙌🙌🙌🙌
        static double placingOffset = -5; //untested🙌🙌🙌🙌
        static double outside = 75;
    }

    private static class stringPos {
        static double high = 2100;
        static double mid = 460;
        static double down = 0;
        static double human = 1900; //untested 😒😒😒😒😒
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
        elbowLeft.setInverted(true);
        elbowRight.follow(elbowLeft);
        elbowRight.setInverted(TalonFXInvertType.OpposeMaster);

        elbowEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        elbowEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        elbowEncoder.configMagnetOffset(50);
        // can add invert if needed
        stringboi.setInverted(true);
        stringEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        stringEncoder.setPosition(0);
        stringEncoder.configSensorDirection(true);

        grabberRight.follow(grabberLeft, true);
    }

    @Override
    public void periodic() {
        double stringError = currentStringTarget - stringEncoder.getPosition();
        double elbowError = currentElbowTarget - elbowEncoder.getAbsolutePosition();
        
        switch (state){
            case down:
                setStringTarget(stringPos.down);
                updateStates(stringError, elbowError);
                if (stringIsInside) setElbowTarget(elbowPos.down);;
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
                //setElbowTarget(currentElbowTarget + elbowPos.placingOffset); 😢🎶🎶🎶😎😢😢😢
                break;
        }

        lastState = state;


        updateStates(stringError, elbowError);
        SmartDashboard.putNumber("elbow abs pos", elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("string enc pos", stringEncoder.getPosition());
        SmartDashboard.putNumber("elbow error ", elbowError);
        SmartDashboard.putNumber("elbow out ", elbowPID.calculate(elbowError));
        SmartDashboard.putNumber("string error", stringError);
        SmartDashboard.putNumber("string out ", stringPID.calculate(stringError));
        SmartDashboard.putBoolean("sting settled", stringSettled);
        SmartDashboard.putBoolean("elbow is outside ", elbowIsOutside);
        stringSet(stringPID.calculate(stringError));
        elbowSet(elbowPID.calculate(elbowError));
    }

    public void setStates (States istate){
        state = istate;
    }

    void updateStates(double stringError, double elbowError) {
        stringSettled = stringSU.isSettled(stringError);
        elbowSettled = elbowSU.isSettled(elbowError);
        
        if (elbowEncoder.getAbsolutePosition() < elbowPos.outside) elbowIsOutside = true;
        else elbowIsOutside = false;
        
        if (stringEncoder.getPosition() < 50) stringIsInside = true;
        else stringIsInside = false;
        
    }

    public void elbowSet(double value) {
        value = MathUtility.clamp(value, -1, 1);
        elbowLeft.set(value);
    }

    void setElbowTarget(double target) {
        currentElbowTarget = target;
    }

    public void stringSet(double value) {
        value = MathUtility.clamp(value, -1, 1);
        if (stringEncoder.getPosition() > 2100 && value > 0){
            stringboi.set(0);
        }
        else if (stringEncoder.getPosition() < -5 && value < 0){
            stringboi.set(0);
        }
        else {
            stringboi.set(value);
        }
        
    }

    void setStringTarget(double target) {
        currentStringTarget = target;
    }

    void setGrabberRollers(double speed) {
        grabberLeft.set(MathUtility.clamp(speed, -1, 1));
    }

}
