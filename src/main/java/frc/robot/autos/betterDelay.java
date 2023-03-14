package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class betterDelay extends CommandBase {
  double time; 
  double TotalTime;
  double x;
  double dt = Timer.getFPGATimestamp() - time;

  public betterDelay(double seconds) {
    TotalTime = seconds;
  }

  public void initialize(){
    time = Timer.getFPGATimestamp();
  }
  
  public void execute(){
        dt = Timer.getFPGATimestamp() - time;
  }

  public void end(boolean interrupted){
  }

  public boolean isFinished(){
      if (dt > TotalTime) return true;
      else return false;
  }
}

