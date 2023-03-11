package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
    private CANSparkMax leftIntake = new CANSparkMax(Constants.intake.leftMotorPort, MotorType.kBrushless);
    private CANSparkMax rightIntake = new CANSparkMax(Constants.intake.rightMotorPort, MotorType.kBrushless);
    private Solenoid intakeClamp = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.intake.solenoidPort);

    private WPI_TalonFX tilter = new WPI_TalonFX(Constants.intake.tilterPort, "7130");
    private PID tilterPID = new PID(0.0000, 0, 0, 0, 0);
    private double currentTilterTarget = tilterPos.up;
    private DigitalInput limitSwitch = new DigitalInput(1);

    private double lastTime;
    private double elapsedTime = 0;
    
    
    private boolean isClampped = true;
    private int posIndex = 2;
    private int lastPosIndex = 2;
    private double intakeSpeed = 0;
    private boolean shoot = false;


    static class tilterPos{
        public static double up = 0;
        public static double down = -32000;
        public static double second = -11000;
    }

    static enum ShootMode {
        mid,
        high,
        full,
    }

    ShootMode mode = ShootMode.high;

    public Intake(){
        rightIntake.setInverted(true);
        tilter.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTime;

        setTilterPos(posIndex);
  
        if (posIndex == 0 && !shoot){
            intakeSpeed = 0.5;
        }
        if (posIndex == 1 && shoot){
            intakeSpeed = 0.2;
            elapsedTime += dt;
            if (elapsedTime >= 0.1) {
                switch (mode){
                    case high:
                        intakeSpeed = -0.55;
                        break;
                    case mid:
                        intakeSpeed = -0.35;
                        break;
                    case full:
                        intakeSpeed = -1;
                        break;
                }
            }
            if (elapsedTime >= 1){
                shoot = false;
                elapsedTime = 0;
            } 
        }
        if (posIndex != 0 && !shoot) {
            //clamp();
            intakeSpeed = 0;
        }

        if (lastPosIndex == 0 && posIndex != 0){
            //clamp();
        }
        if (isClampped) intakeClamp.set(false);
        if (!isClampped) intakeClamp.set(true);
        //setRollers(intakeSpeed);
        double tilterError = currentTilterTarget - tilter.getSelectedSensorPosition();
        double out = tilterPID.calculate(tilterError);
        out = MathUtility.clamp(out, -1, 1);
        tilterSet(out);
        
        if (currentTilterTarget == tilterPos.up){
            //currentTilterTarget = 0;
        }
        if (limitSwitch.get()){
            tilter.setSelectedSensorPosition(0);
        }
        lastTime = currentTime;
        lastPosIndex = posIndex;
        SmartDashboard.putBoolean("tileter limit", limitSwitch.get());
        SmartDashboard.putNumber("tilterPos", tilter.getSelectedSensorPosition());
        SmartDashboard.putNumber("tilterError", tilterError);
        SmartDashboard.putNumber("tilter out ", out);
        SmartDashboard.putNumber("posindex", posIndex);
        SmartDashboard.putBoolean("isClamoed", isClampped);
        SmartDashboard.putNumber("currentTarget", currentTilterTarget);
    }

    public void tilterSet (double speed){
        speed = MathUtility.clamp(speed, -1, 1);
        if (limitSwitch.get() && speed > 0) {
            tilter.set(0);
        }
        else tilter.set(speed);
    }


    public void setTilterPos(int iposIndex){
        posIndex = MathUtility.clamp(iposIndex, 0, 2);
        if (!isClampped) return;
        switch (posIndex){
            case 0: {
                currentTilterTarget = tilterPos.down;
                break;
            }
            case 1: {
                currentTilterTarget = tilterPos.second;
                break;
            }
            case 2: {
                currentTilterTarget = tilterPos.up;
                break;
            }
        }
    }

    public void setRollers(double speed){
        speed = MathUtility.clamp(speed, -1, 1);
        leftIntake.set(speed);
        rightIntake.set(speed);
    }

    public void clamp(){
        isClampped = true;
    }

    public void unClamp(){
        if (posIndex != 0) return;
        isClampped = false;
    }

    public void shoot(){
        shoot = true;   
    }

    public void setState(int currentTilterPos, boolean shoot){
        posIndex = currentTilterPos;
        this.shoot = shoot;
        setTilterPos(posIndex);
    }
    
}
