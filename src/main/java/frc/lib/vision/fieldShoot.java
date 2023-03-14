package frc.lib.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class fieldShoot {
    Transform3d robot2Apriltag;
    static double yDeadband = 1;//in meters 😱😱 haven't tested

    public static boolean OKshoot(Transform3d toApriltag){
        if(Math.abs(toApriltag.getY()) <= yDeadband){
            return true;
        }else{
            return false;
        }
    }
}
