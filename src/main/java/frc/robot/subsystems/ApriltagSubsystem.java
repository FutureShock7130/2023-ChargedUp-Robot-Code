package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ImageConstants;

public class ApriltagSubsystem extends SubsystemBase {


    public ApriltagSubsystem() {}
  
  PhotonCamera PVCamera = new PhotonCamera("Limelight1");
  PhotonPipelineResult mResult = new PhotonPipelineResult();
  boolean mHasTarget;
  List<PhotonTrackedTarget> mTargetList = mResult.getTargets();

  //define target
  PhotonTrackedTarget mTarget = mResult.getBestTarget();

  //define information of the target
  double mYaw;
  double mPitch;
  double mArea;
  double mSkew;
  double mPoseAmbiguity;
  Transform3d mBestCameraToTarget;
  Transform3d mAlternateCameraToTarget;
  List<TargetCorner> mCorners;

  /**
   * Define the position variable in meters
   */
  double mx, my;
  int mTargetID;
  Translation3d mApriltagIDPosition;
  Transform3d targetToCamera;
  Transform3d targetToRonotCenter;

  //variables of getAccuracy()
  double ambiguitySum;

  

  @Override
  public void periodic() {
    mResult = PVCamera.getLatestResult();
    mHasTarget = mResult.hasTargets();

    if( mHasTarget ){
      mTarget = mResult.getBestTarget();
      mYaw = mTarget.getYaw();
      mPitch = mTarget.getPitch();
      mArea = mTarget.getArea();
      mSkew = mTarget.getSkew();
      mTargetID = mTarget.getFiducialId();
      mPoseAmbiguity = mTarget.getPoseAmbiguity();

      mBestCameraToTarget = mTarget.getBestCameraToTarget();
      mCorners = mTarget.getDetectedCorners();
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  public void snapshot(){
    PVCamera.takeInputSnapshot();
    PVCamera.takeOutputSnapshot();
  }
  
  /**
   * The specific coordination is in Constants.FieldConstants
   * @return the translation3d of the center of your robot on the field in meters 
   */
  public Translation3d getPosByApriltag(){

    mTargetID = getTargetID();
    Translation3d robotCenterPosition = new Translation3d();

    if( mTargetID != 0 ){
        mApriltagIDPosition = FieldConstants.ApriltagMap.get(mTargetID);
        targetToCamera = getCameratoTarget().inverse();
        targetToRonotCenter = targetToCamera.plus(ImageConstants.CameraToRobotCenter);

        if( mTargetID <= 4 ){
          robotCenterPosition = new Translation3d (
            mApriltagIDPosition.getX() - ImageConstants.kXDis * targetToRonotCenter.getX(),
            mApriltagIDPosition.getY() - ImageConstants.kYDis * targetToRonotCenter.getY(),
            mApriltagIDPosition.getZ() + ImageConstants.kZDis * targetToRonotCenter.getZ()
          );
        }else{
          robotCenterPosition = new Translation3d (
            mApriltagIDPosition.getX() + ImageConstants.kXDis * targetToRonotCenter.getX(),
            mApriltagIDPosition.getY() + ImageConstants.kYDis * targetToRonotCenter.getY(),
            mApriltagIDPosition.getZ() + ImageConstants.kZDis * targetToRonotCenter.getZ()
          );
        }

        mTargetID = 0;
    }

    SmartDashboard.putNumber("apriltag_posX", robotCenterPosition.getX());
    SmartDashboard.putNumber("apriltag_posY", robotCenterPosition.getY());
    SmartDashboard.putNumber("apriltag_posZ", robotCenterPosition.getZ());

    return robotCenterPosition;
  }

  /**
   * in meters
   * @return
   * x -> front</p>
   * y -> left</p>
   * z -> up</p>
   **/
  public Transform3d getCameratoTarget(){
    return hasTarget() ? mBestCameraToTarget : new Transform3d( new Translation3d(0,0,0), new Rotation3d(0,0,0));
  }

/*
 * If there's no target spooted then the default value is zero.
 */
  public int getTargetID(){
    return hasTarget() ? mTargetID : 0;
  }

  /**
   * If there's no target spooted then the default value is zero.
   * @return degrees
   **/
  public double getYaw(){
    return hasTarget() ? mYaw : 0;
  }

  /**
   * If there's no target spooted then the default value is zero.
   * @return degrees
   **/
  public double getSkew(){
    return hasTarget() ? mSkew : 0;
  }

  /**
   * If there's no target spooted then the default value is zero.
   * @return degrees
   **/
  public double getPitch(){
    return hasTarget() ? mPitch : 0;
  }

  public boolean hasTarget(){
    return mHasTarget;
  }

/**
 * If there's no target spooted then the default value is zero.</p>
 * When the robot is near the apriltag there will be a very high possibility that the ambiguity return zero😱😱
 */
  public double getPoseAmbiguity(){
    return hasTarget() ? mPoseAmbiguity : 0;
  }

  public void switchPipelines(int index){
    PVCamera.setPipelineIndex(index);
  }

  public void switchDriverMode(boolean enabled){
    PVCamera.setDriverMode(enabled);
  }

  public int getPipeline(){
    return PVCamera.getPipelineIndex();
  }

  public boolean getDriverMode(){
    return PVCamera.getDriverMode();
  }

  Timer clock = new Timer();

  /**
   * 
   * @return a value [0,1] defining the accuracy of a apriltag return data
   */
  public double getAccuracy(){
    clock.start();
    if (mPoseAmbiguity > 0.2){
      ambiguitySum += Math.abs(0.2 - mPoseAmbiguity);
    }
    if (clock.advanceIfElapsed(1)){
      clock.stop();
      clock.reset();
    }
    return (4 - ambiguitySum)/4 ;
  }
}