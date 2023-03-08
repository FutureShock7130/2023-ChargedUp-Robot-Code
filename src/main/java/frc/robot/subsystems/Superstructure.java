// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FollowerType;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.ChenryLib.PID;
// import frc.ChenryLib.SettledUtility;
// import frc.robot.Constants;

// public class Superstructure extends SubsystemBase{
//     private WPI_TalonFX elbowLeft;
//     private WPI_TalonFX elbowRight;
//     private CANCoder elbowEncoder;
//     private SettledUtility elbowSU;

//     private WPI_TalonFX stringboi;
//     private CANCoder stringEncoder;
//     private SettledUtility stringSU;

//     private CANSparkMax rotationboi;
//     private RelativeEncoder rotationEncoder;
//     private SettledUtility rotationSU;

//     private Solenoid squishyboi;
    

//     private PID elbowPID;
//     private PID stringPID;
//     private PID rotationPID;


//     private double currentElbowTarget = 0;
//     private double currentStringTarget = 0;
//     private double currentRotationTarget = 0;

//     boolean isIdle = true;

//     boolean stringSettled = true;
//     boolean elbowSettled = true;
//     boolean rotationSettled = true;

//     private static class rotationPos {
//         static double standby = 0;
//         static double horizontal = 0;
//         static double flippedHorizontal = 0;
//     }

//     private static class elbowPos{
//         static double mid = 0;
//         static double up = 0;
//         static double down = 0;
//         static double standbyIntake = 0;
//         static double standbyPut = 0;
//     }

//     private static class stringPos{
//         static double up = 0;
//         static double mid = 0;
//         static double down = 0;
//         static double standbyIntake = 0;
//     }


//     enum overallStates{
//         standbyIntake,
//         standbyPut,
//         moving,
//         scoring
//     }

//     Superstructure(){
//         elbowLeft = new WPI_TalonFX(Constants.Superstructure.talonLeftPort);
//         elbowRight = new WPI_TalonFX(Constants.Superstructure.talonRightPort);
//         elbowRight.follow(elbowLeft, FollowerType.PercentOutput);
//         elbowEncoder = new CANCoder(Constants.Superstructure.elbowCanCoderPort);
//         elbowPID = new PID(0.001, 0, 0, 0, 0);

//         overallStates states = overallStates.standbyIntake;
        
//         stringboi = new WPI_TalonFX(Constants.Superstructure.talonStringPort);
//         stringPID = new PID(0, 0, 0, 0, 0);
//         rotationboi = new CANSparkMax(Constants.Superstructure.neoRotationPort, MotorType.kBrushless);
//         rotationEncoder = rotationboi.getEncoder();
//         rotationPID = new PID(0, 0, 0, 0, 0);

//         squishyboi = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Superstructure.solenoidPort);
//     }
    
//     @Override
//     public void periodic() {
//         double stringError = currentStringTarget - stringEncoder.getPosition();
//         double rotationError = currentRotationTarget - rotationEncoder.getPosition();
//         double elbowError = currentElbowTarget - elbowEncoder.getAbsolutePosition();
//         updateStates(rotationError, stringError, elbowError);
        
//         switch (){
//             case (){
//                 scoreStandbySequence();
//             }
//             case () {
//                 scoreSequence();
//             }
//         }


//         rotationSet(rotationPID.calculate(rotationError));
//         stringSet(stringPID.calculate(stringError));
//         elbowSet(elbowPID.calculate(elbowError));
//     }

//     void standbyIntakeSequence(){
//         if (!isIdle) return;
//         isIdle = false;
//         boolean running = true;
//         rotationSetTarget(rotationPos.standby);
//         stringSetTarget(stringPos.standbyIntake);
//         elbowSetTarget(elbowPos.standbyIntake);
//         unClamp();
//         running = false;
//         if (!running) isIdle = true;
//     }

//     void scoreStandbySequence(){
        
//         isIdle = false;
//         rotationSetTarget(rotationPos.standby);
//         stringSetTarget(stringPos.standbyIntake);
//         if (stringSettled) elbowSetTarget(elbowPos.standbyIntake);
//         if (stringSettled && elbowSettled) clamp();
//         elbowSetTarget(elbowPos.standbyPut);
//         isIdle = true;
//     }

//     void scoreSequence(){
//         isIdle = false;
//         elbowSetTarget(elbowPos.up);
//         if (elbowSettled) stringSetTarget(stringPos.up);
//         isIdle = true;
//     }

//     void updateStates(double rotationError, double stringError, double elbowError){
//         stringSettled = stringSU.isSettled(stringError);
//         rotationSettled = rotationSU.isSettled(rotationError);
//         elbowSettled = elbowSU.isSettled(elbowError);
//     }

//     void elbowSet(double value){
//         elbowLeft.set(ControlMode.PercentOutput, value);
//     }

//     void elbowSetTarget(double target){
//         currentElbowTarget = target;
//     }

//     void stringSet(double value){
//         stringboi.set(ControlMode.PercentOutput, value);
//     }

//     void stringSetTarget(double target){
//         currentStringTarget = target;
//     }

//     void rotationSet (double value){
//         rotationboi.set(value);
//     }

//     void rotationSetTarget (double target){
//         currentRotationTarget = target;
//     }

//     void clamp (){
//         squishyboi.set(true);
//     }

//     void unClamp(){
//         squishyboi.set(false);
//     }



// }
