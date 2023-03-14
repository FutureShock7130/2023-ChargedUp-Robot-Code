// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Constants.intake;
import frc.robot.autos.betterDelay;

public class Index extends SubsystemBase {

  private CANSparkMax leftRollers = new CANSparkMax(Constants.intake.leftMotorPort, MotorType.kBrushless);
  private CANSparkMax rightRollers = new CANSparkMax(Constants.intake.rightMotorPort, MotorType.kBrushless);
  private Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, intake.solenoidPort);
  private WPI_TalonFX tilter = new WPI_TalonFX(intake.tilterPort, "7130");
  private DigitalInput limitSwitch = new DigitalInput(1);

  private PID tilterPID = new PID(0.00005, 0, 0.00005, 0, 0);

  private boolean isClamped = true;
  private boolean shoot = false;

  private indexStates state = indexStates.AimTop;
  // private indexPos position = indexPos.Up;
  

  public enum indexStates {
    Indexing,
    Standby,
    AimTop,
    AimMiddle,
    AimBottom,
    AimTopCS,
    AimMiddleCS,
    AimBottomCS
  }

  public enum indexPos {
    Up, // Up Limit
    Down, // Down Limit
    Top, // Top Row
    Middle, // Middle Row
    Bottom, // Bottom Row
    TopCS, // Top Row (Charge Station)
    MiddleCS, // Middle Row (Charge Station)
    BottomCS // Bottom Row (Charge Station)
  }

  // Tested Pos
  private static class tilterPos {
    static double upLimit = 0;
    static double upSmoothPoint = -3564;
    static double downlimit = -27968;
    static double topRow = 0;
    static double middleRow = 0;
    static double bottomRow = 0;
    static double topRowCS = 0;
    static double middleRowCS = 0;
    static double bottomRowCS = 0;
  }

  private double currentTilterPos;

  /** Creates a new Index. */
  public Index() {
    rightRollers.follow(leftRollers, true);
    tilter.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (limitSwitch.get()) tilter.setSelectedSensorPosition(0);
    currentTilterPos = tilter.getSelectedSensorPosition();
    // updateStates();
    if (shoot) shoot(-1);
    if (limitSwitch.get() && tilter.getSelectedSensorVelocity() > 0) tilter.set(0);
    if (tilter.getSelectedSensorPosition() <= tilterPos.downlimit && tilter.getSelectedSensorVelocity() < 0) tilter.set(0);

    SmartDashboard.putBoolean("Intake isClamped", isClamped);
    SmartDashboard.putBoolean("Intake atUpLimit", limitSwitch.get());
    SmartDashboard.putString("Intake state", state.toString());
    SmartDashboard.putNumber("Intake Pos", tilter.getSelectedSensorPosition());
  }

  public void setRollers(double speed) {
    leftRollers.set(MathUtility.clamp(speed, -1, 1));
  }

  public void shoot(double speed) {
    setRollers(0.3);
    new betterDelay(0.5);
    setRollers(MathUtility.clamp(speed, -1, 1));
    new betterDelay(1);
    shoot = false;
  }

  public void clamp() {
    solenoid.set(false);
    isClamped = true;
  }

  public void unclamp() {
    solenoid.set(true);
    isClamped = false;
  }

  public Command toggleClamp() {
    return new InstantCommand(() -> {
      if (isClamped == false) {
        clamp();
      } else if (isClamped == true) {
        unclamp();
      }
    }, this);
  }

  public void setTilterPosBySpd(double speed) {

    if (speed > 0 && limitSwitch.get()) 
      tilter.set(ControlMode.PercentOutput, 0);
    // else if (speed < 0 && tilter.getSelectedSensorPosition() < tilterPos.downlimit) 
    //   tilter.set(ControlMode.PercentOutput, 0);
    else if (tilter.getSelectedSensorPosition() > tilterPos.upSmoothPoint) 
      tilter.set(ControlMode.PercentOutput, MathUtility.clamp(speed * 0.4, -1, 1));
    else if (tilter.getSelectedSensorPosition() < tilterPos.downlimit && speed < 0)
      tilter.set(ControlMode.PercentOutput, 0);
    else 
      tilter.set(ControlMode.PercentOutput, MathUtility.clamp(speed, -1, 1));
  }

  public void stick() {
    double setpoint = 0;
    double error = tilter.getSelectedSensorPosition() - setpoint;
    double output = tilterPID.calculate(error);
    tilter.set(ControlMode.PercentOutput, MathUtility.clamp(error, -0.1, 0.1));
  }

  public void setTilterPosAuto(indexPos pos) {
    switch (pos) {
      case Up:
        tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterPos.upLimit - currentTilterPos));
        break;
      case Down:
        tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterPos.downlimit - currentTilterPos));
        break;
      case Top:
        tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterPos.topRow - currentTilterPos));
        break;
      case Middle:
        tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterPos.middleRow - currentTilterPos));
        break;
      case Bottom:
        tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterPos.bottomRow - currentTilterPos));
        break;
      case TopCS:
        tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterPos.topRowCS - currentTilterPos));
        break;
      case MiddleCS:
        tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterPos.middleRowCS - currentTilterPos));
        break;
      case BottomCS:
        tilter.set(ControlMode.PercentOutput, tilterPID.calculate(tilterPos.bottomRowCS - currentTilterPos));
        break;
      default:
        break;
    }
  }

  public void updateStates() {
    switch (state) {
      case Indexing:
        setTilterPosAuto(indexPos.Down);
        new betterDelay(0.2);
        unclamp();
        setRollers(0.28);
        break;
      case Standby:
        clamp();
        setRollers(0);
        new betterDelay(0.2);
        setTilterPosAuto(indexPos.Up);
        break;
      case AimTop:
        clamp();
        setRollers(0);
        new betterDelay(0.2);
        setTilterPosAuto(indexPos.Top);
        new betterDelay(0.2);
        if (shoot) shoot(-1);
        break;
      case AimMiddle:
        clamp();
        setRollers(0);
        new betterDelay(0.2);
        setTilterPosAuto(indexPos.Middle);
        new betterDelay(0.2);
        if (shoot) shoot(-1);
        break;
      case AimBottom:
        clamp();
        setRollers(0);
        new betterDelay(0.2);
        setTilterPosAuto(indexPos.Bottom);
        new betterDelay(0.2);
        if (shoot) shoot(-1);
        break;
      case AimTopCS:
        clamp();
        setRollers(0);
        new betterDelay(0.2);
        setTilterPosAuto(indexPos.TopCS);
        new betterDelay(0.2);
        if (shoot) shoot(-1);
        break;
      case AimMiddleCS:
        clamp();
        setRollers(0);
        new betterDelay(0.2);
        setTilterPosAuto(indexPos.MiddleCS);
        new betterDelay(0.2);
        if (shoot) shoot(-1);
        break;
      case AimBottomCS:
        clamp();
        setRollers(0);
        new betterDelay(0.2);
        setTilterPosAuto(indexPos.BottomCS);
        new betterDelay(0.2);
        if (shoot) shoot(-1);
        break;
      default:
        break;
    }
  }

  public void setState(indexStates state, boolean shoot) {
    this.state = state;
    this.shoot = shoot;
  }

  public indexStates getState() {
    return state;
  }

}
