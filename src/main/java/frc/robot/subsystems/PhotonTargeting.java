// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** April Tag Id's 10.5 9
 * Family: 36h11
 * Subwoofer Height: 4' 3 7/8 + 5.25 (same as source)
 * Sub Center: r4 b7
 * offset 1'5" + 5.25
 * Red Sub Right: 3 Blue Sub Left: 8
 * 
 * Amp Height: 4" 1/8" + 5.25
 * Amp: r5 b6
 * 
 * Source Height: 4' 3 7/8 + 5.25 (same as subwoofer)
 * offset from center: 1' 7 3/8 + 5.25
 * Source Left: r10 b2
 * Source Right: r9 b1
 * 
 * Stage Height: 3'11.5" + 4.5
 * Stage Back: r13 b14, Left r11 b15, Right: r12 b16
 * 
 * Setup pipelines for the following id's red first, then blue, last one reserved for unforseen
 * 4,3,5,10,13 7,8,6,2,16
 * speaker, alt speaker, amp, source left, stage back.  
 */

public class PhotonTargeting extends SubsystemBase {
  private final double HEIGHT_OF_SPEAKER = 57.125; //use this for source as well
  private final double HEIGHT_OF_AMP = 53.375; 
  private final double HEIGHT_OF_STAGE = 52; 
  private final double HEIGHT_OF_CAMERA = 12;
  private final double CAMERA_MOUNT_ANGLE = 45.2;
  private PhotonCamera shooterCamera;

  private double validTarget;
  List<PhotonTrackedTarget> targets;
  private double x;
  private double y;
  private int whichTarget;
  private final String[] TARGETING_WHAT = {"Blue Source L" , "Blue Source R" , "Red Alt Speaker" , "Red Speaker" , "Red Amp" , "Blue Amp" , "Blue Speaker" , "Blue Alt Speaker" , 
                                    "Red Source R" , "Red Source L" , "Red Stage L" , "Red Stage R" , "Red Stage B" , "Blue Stage B" , "Blue Stage L" , "Blue Stage R"};
  private final int[] redTargets = {4,3,5,10,13,12,11,9};
  private final int[] blueTargets = {7,8,6,2,14,16,15,1};
  private int[] pipelines;
                                  



  /** Creates a new PhotonTargeting. */
  public PhotonTargeting() {
    shooterCamera = new PhotonCamera("shooterCamera");
    validTarget = -1;
    x = 333;
    y = 333;
    whichTarget = 4;
  }

  @Override
  public void periodic() {
    var result = shooterCamera.getLatestResult();
    if(result.hasTargets()){
      validTarget = 1;
      targets = result.getTargets();
    }
    SmartDashboard.putNumber("Valid Target", validTarget);
    SmartDashboard.putNumber("Target x", x);
    SmartDashboard.putNumber("Target y", y);
    SmartDashboard.putString("Targeting", TARGETING_WHAT[whichTarget - 1]);
    SmartDashboard.putNumber("range", calcRange());


    // This method will be called once per scheduler run
  }

  //0 speaker, 1 altSpeaker, 2 amp, 3 source left, 4 stage back, stage Right, Stage left, source right
  public void changeTag(int whichTarget){//set according to which tag to track
    this.whichTarget = whichTarget;
  }

  public void setSide(boolean isRed){
    if(isRed){
      //pipelines = redPipelines;
    }
    else{
      //pipelines = bluePipelines;
    }
    changeTag(whichTarget);
  }
  public boolean getSide(){
    return sideChooser.getSelected();
  }

  public double getValidTarget(){
    return validTarget;
  }
  
  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }




}
