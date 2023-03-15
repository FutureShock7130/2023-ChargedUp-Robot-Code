// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  private Timer time;

  private boolean isClamped = true;
  private boolean shoot = false;
  private indexStates state = indexStates.Standby;
  private indexPos desirePos = indexPos.Up;

  public enum indexStates {
    Indexing,
    IndexHuman,
    AimTop,
    AimMiddle,
    AimBottom,
    Standby
  }

  public enum indexPos {
    Up, // Up Limit
    Down, // Down Limit
    GridCom // In Community
  }

  // Tested Pos
  private static class tilterPos {
    static double upLimit = 0;
    static double upSmoothPoint = -3564;
    static double downlimit = -26000;
    static double inCom = -6900;
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
    updateStates();
    if (shoot) shoot(-1);
    if (limitSwitch.get() && tilter.getSelectedSensorVelocity() > 0) tilter.set(0);
    if (tilter.getSelectedSensorPosition() <= tilterPos.downlimit && tilter.getSelectedSensorVelocity() < 0) tilter.set(0);
    if (desirePos == indexPos.Up && !limitSwitch.get() && tilterPos.upLimit - currentTilterPos < 2500) tilter.set(0.15);

    SmartDashboard.putBoolean("Intake isClamped", isClamped);
    SmartDashboard.putBoolean("Intake atUpLimit", limitSwitch.get());
    SmartDashboard.putString("Intake state", state.toString());
    SmartDashboard.putNumber("Intake Pos", tilter.getSelectedSensorPosition());
    SmartDashboard.putNumber("Intake Vel", tilter.getSelectedSensorVelocity());
  }

  public void setRollers(double speed) {
    leftRollers.set(MathUtility.clamp(speed, -1, 1));
  }

  public void shoot(double speed) {
    setRollers(MathUtility.clamp(speed, -1, 1));
    new betterDelay(1);
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
    double error = setpoint - tilter.getSelectedSensorPosition();
    double output = tilterPID.calculate(error);
    tilter.set(ControlMode.PercentOutput, MathUtility.clamp(output, -0.17, 0.17));
  }

  public void setTilterPosAuto(indexPos pos) {
    switch (pos) {
      case Up:
        desirePos = indexPos.Up;
        tilter.set(ControlMode.PercentOutput, MathUtility.clamp(tilterPID.calculate(tilterPos.upLimit - currentTilterPos), -0.3, 0.3));
        break;
      case Down:
        desirePos = indexPos.Down;
        tilter.set(ControlMode.PercentOutput, MathUtility.clamp(tilterPID.calculate(tilterPos.downlimit - currentTilterPos), -0.27, 0.27));
        break;
      case GridCom:
        desirePos = indexPos.GridCom;
        tilter.set(ControlMode.PercentOutput, MathUtility.clamp(tilterPID.calculate(tilterPos.inCom - currentTilterPos), -0.17, 0.17));
      default:
        break;
    }
  }

  public void updateStates() {
    switch (state) {
      case Indexing:
        setTilterPosAuto(indexPos.Down);
        setRollers(0.28);
        if (MathUtility.isWithin(tilter.getSelectedSensorPosition(), tilterPos.downlimit - 1000, tilterPos.downlimit + 1000)) unclamp();
        break;
      case IndexHuman:
        clamp();
        setRollers(0.28);
        setTilterPosAuto(indexPos.Up);
        break;
      case AimTop:
        clamp();
        setRollers(0);
        setTilterPosAuto(indexPos.GridCom);
        break;
      case AimMiddle:
        clamp();
        setRollers(0);
        setTilterPosAuto(indexPos.GridCom);
        break;
      case AimBottom:
        clamp();
        setRollers(0);
        setTilterPosAuto(indexPos.GridCom);
        break;
      case Standby:
        clamp();
        setRollers(0);
        // setRollers(0.038);
        setTilterPosAuto(indexPos.Up);
      default:
        break;
    }
  }

  public void shootByState() {
    switch (state) {
      case AimTop:
        shoot(-1);
        break;
      case AimMiddle:
        shoot(-0.6);
        break;
      case AimBottom:
        shoot(-0.2);
        break;
    }
  }

  public void setState(indexStates state) {
    this.state = state;
  }

  public indexStates getState() {
    return state;
  }

}
