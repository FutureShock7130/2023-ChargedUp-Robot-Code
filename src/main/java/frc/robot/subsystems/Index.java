// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Constants.intake;

public class Index extends SubsystemBase {

  private CANSparkMax leftRollers = new CANSparkMax(Constants.intake.leftMotorPort, MotorType.kBrushless);
  private CANSparkMax rightRollers = new CANSparkMax(Constants.intake.rightMotorPort, MotorType.kBrushless);
  private Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, intake.solenoidPort);
  private TalonFX tilter = new TalonFX(intake.tilterPort, "7130");
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
    static double downlimit = 0;
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

    // if (currentTilterPos == tilterPos.upLimit) position = indexPos.Up;
    // if (currentTilterPos == tilterPos.downlimit) position = indexPos.Down;
    // if (currentTilterPos == tilterPos.topRow) position = indexPos.Top;
    // if (currentTilterPos == tilterPos.middleRow) position = indexPos.Middle;
    // if (currentTilterPos == tilterPos.bottomRow) position = indexPos.Bottom;
    // if (currentTilterPos == tilterPos.topRowCS) position = indexPos.TopCS;
    // if (currentTilterPos == tilterPos.middleRowCS) position = indexPos.MiddleCS;
    // if (currentTilterPos == tilterPos.bottomRowCS) position = indexPos.BottomCS;

    switch (state) {
      case Indexing:
        setTilterPosAuto(indexPos.Down);
        Timer.delay(0.2);
        unclamp();
        setRollers(-0.28);
        break;
      case Standby:
        clamp();
        setRollers(0);
        Timer.delay(0.2);
        setTilterPosAuto(indexPos.Up);
        break;
      case AimTop:
        clamp();
        setRollers(0);
        Timer.delay(0.2);
        setTilterPosAuto(indexPos.Top);
        Timer.delay(0.2);
        if (shoot) shoot();
        break;
      case AimMiddle:
        clamp();
        setRollers(0);
        Timer.delay(0.2);
        setTilterPosAuto(indexPos.Middle);
        Timer.delay(0.2);
        if (shoot) shoot();
        break;
      case AimBottom:
        clamp();
        setRollers(0);
        Timer.delay(0.2);
        setTilterPosAuto(indexPos.Bottom);
        Timer.delay(0.2);
        if (shoot) shoot();
        break;
      case AimTopCS:
        clamp();
        setRollers(0);
        Timer.delay(0.2);
        setTilterPosAuto(indexPos.TopCS);
        Timer.delay(0.2);
        if (shoot) shoot();
        break;
      case AimMiddleCS:
        clamp();
        setRollers(0);
        Timer.delay(0.2);
        setTilterPosAuto(indexPos.MiddleCS);
        Timer.delay(0.2);
        if (shoot) shoot();
        break;
      case AimBottomCS:
        clamp();
        setRollers(0);
        Timer.delay(0.2);
        setTilterPosAuto(indexPos.BottomCS);
        Timer.delay(0.2);
        if (shoot) shoot();
        break;
      default:
        break;
    }

    if (shoot) shoot();
  }

  public void setRollers(double speed) {
    leftRollers.set(MathUtility.clamp(speed, -1, 1));
  }

  public void shoot() {
    setRollers(-0.3);
    Timer.delay(0.5);
    setRollers(1);
    Timer.delay(1.3);
    shoot = false;
  }

  public void clamp() {
    solenoid.set(true);
    isClamped = true;
  }

  public void unclamp() {
    solenoid.set(false);
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
    else if (speed < 0 && tilter.getSelectedSensorPosition() < tilterPos.downlimit) 
      tilter.set(ControlMode.PercentOutput, 0);
    else 
      tilter.set(ControlMode.PercentOutput, speed);
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

  public void setState(indexStates state, boolean shoot) {
    this.state = state;
    this.shoot = shoot;
  }

}
