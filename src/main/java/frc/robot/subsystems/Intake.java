// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.ChenryLib.MathUtility;
// import frc.ChenryLib.PID;
// import frc.robot.Constants;

// public class Intake extends SubsystemBase{
//     private CANSparkMax leftIntake;
//     private CANSparkMax rightIntake;
//     private Solenoid intakeClamp;
//     private WPI_TalonFX tilter;
//     private PID tilterPID;
//     private double currentTilterTarget;
//     private double lastTime;
//     private double elapsedTime = 0;
//     private int posIndex = 3;
//     private int lastPosIndex = 3;
//     private double intakeSpeed = 0;
//     private boolean shoot = false;
//     static class tilterPos{
//         public static double up = 0;
//         public static double down = -90;
//         public static double third = -45;
//         public static double second = -60;
//     }

//     public Intake(){
//         leftIntake = new CANSparkMax(Constants.intake.leftMotorPort, MotorType.kBrushless);
//         rightIntake = new CANSparkMax(Constants.intake.rightMotorPort, MotorType.kBrushless);
//         rightIntake.setInverted(true);


//         intakeClamp = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.intake.solenoidPort);


//         tilter = new WPI_TalonFX(Constants.intake.tilterPort);
//         tilterPID = new PID(0.0001, 0, 0, 0, 0);
//     }

//     @Override
//     public void periodic() {
//         double currentTime = Timer.getFPGATimestamp();
//         double dt = currentTime - lastTime;
  
//         if (posIndex == 0 && !shoot){
//             intakeSpeed = 1;
//         }
//         if (posIndex == 1 && shoot){
//             shoot = true;
//             intakeSpeed = 0.2;
//             elapsedTime += dt;
//             if (elapsedTime >= 0.1) intakeSpeed = -0.5;
//             if (elapsedTime >= 0.5){
//                 shoot = false;
//                 elapsedTime = 0;
//             } 
//         }
//         if (posIndex == 2 && shoot){
//             shoot = true;
//             intakeSpeed = 0.2;
//             elapsedTime += dt;
//             if (elapsedTime >= 0.1) intakeSpeed = -0.9;
//             if (elapsedTime >= 0.5){
//                 shoot = false;
//                 elapsedTime = 0;
//             } 
//         }
//         else if (posIndex != 0 && !shoot) intakeSpeed = 0;

//         if (lastPosIndex == 0 && posIndex != 0){
//             clamp();
//         }
//         setRollers(intakeSpeed);
//         double tilterError = currentTilterTarget - tilter.getSelectedSensorPosition();
//         tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterError));
        
//         lastTime = currentTime;
//         lastPosIndex = posIndex;
//         SmartDashboard.putNumber("tilterPos", tilter.getSelectedSensorPosition());
//         SmartDashboard.putNumber("tilterError", tilterError);
//     }

//     public void setTilterTarget(double target){
//         currentTilterTarget = target;
//     }

//     public void setTilterPos(int iposIndex){
//         posIndex = (int) MathUtility.clamp(iposIndex, 0, 3);
//         switch (posIndex){
//             case 0: {
//                 setTilterTarget(tilterPos.down);
//             }
//             case 1: {
//                 setTilterTarget(tilterPos.second);
//             }
//             case 2: {
//                 setTilterTarget(tilterPos.third);
//             }
//             case 3: {
//                 setTilterTarget(tilterPos.up);
//             }
//         }
//     }

//     public void setRollers(double speed){
//         leftIntake.set(speed);
//         rightIntake.set(speed);
//     }

//     public void clamp(){
//         intakeClamp.set(true);
//     }

//     public void unClamp(){
//         intakeClamp.set(false);
//     }

//     public void shoot(){
//         shoot = true;   
//     }

//     public void setState(int currentTilterPos, boolean shoot){

//     }
    
// }
