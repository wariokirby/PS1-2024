// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
 * 4,3,5,10,13 7,8,6,2,14
 * speaker, alt speaker, amp, source left, stage back.  
 */

public class Targeting extends SubsystemBase {
  //distances now in inches
  private final double HEIGHT_OF_SPEAKER = 53.5; //57.125 use this for source as well
  private final double HEIGHT_OF_AMP = 53.125; 
  private final double HEIGHT_OF_STAGE = 52; 
  private final double HEIGHT_OF_CAMERA = 12;
  private final double CAMERA_MOUNT_ANGLE = 25.25;//43, 35.8

  private NetworkTable limelight;
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;

  private double validTarget;
  private double x;
  private double y;

  private int whichTarget;
  private boolean usingAlt;
  private SendableChooser<Boolean> sideChooser;
  private final int[] redPipelines = {0,1,2,3,4};
  private final int[] bluePipelines = {5,6,7,8,9};
  private int[] pipelines;

  /** Creates a new Targeting. */
  public Targeting() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    tv = limelight.getEntry("tv");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");

    //the following are out of range indicating no target found
    validTarget = 0;
    x = 100;
    y = 100;

    whichTarget = 0;
    usingAlt = false;
    sideChooser = new SendableChooser<>();
    sideChooser.setDefaultOption("Blue", false);
    sideChooser.addOption("red", true);
    SmartDashboard.putData("red or Blue", sideChooser);
    pipelines = bluePipelines;
    limelight.getEntry("pipeline").setNumber(pipelines[whichTarget]);

    limelight.getEntry("ledMode").setNumber(1);
    limelight.getEntry("camMode").setNumber(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    validTarget = tv.getDouble(0);
    if(whichTarget == 0){
      if(validTarget == 0){
        ledOn(0);
        if(usingAlt){
          changeTag(0);
          usingAlt = false;
        }
        else{
          changeTag(1);
          usingAlt = true;
        }
      }
      else{
        if(calcRange() < 90){
          ledOn(2);
        }
        else{
          ledOn(1);
        }
      }
    }
    x = tx.getDouble(0);
    y = ty.getDouble(0);

    SmartDashboard.putBoolean("Valid Target", validTarget > 0);
    SmartDashboard.putNumber("Target x", x);
    SmartDashboard.putNumber("Target y", y);
    SmartDashboard.putNumber("range", calcRange());
    setSide(sideChooser.getSelected());
  }

  //0 speaker, 1 altSpeaker, 2 amp, 3 source left, 4 stage back
  public void changeTag(int whichTarget){//set according to which tag is in which pipeline
    if(whichTarget == 1){
      this.whichTarget = 0;
    }
    else{
      this.whichTarget = whichTarget;
    }
    limelight.getEntry("pipeline").setNumber(pipelines[whichTarget]);
  }

  public void setSide(boolean isRed){
    if(isRed){
      pipelines = redPipelines;
    }
    else{
      pipelines = bluePipelines;
    }
    changeTag(whichTarget);
  }
  public boolean getSide(){//red is true
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


  public void ledOn(int on) {
    NetworkTableEntry led = limelight.getEntry("ledMode");
    if(on == 2) {
      led.setNumber(3);
    }
    else if(on ==  1){//blink
      led.setNumber(2);
    } 
    else {
      led.setNumber(1);
    }
  }

  //calculate horizontal range to target
  public double calcRange() {
    double height;
    if(whichTarget == 0){
      height = HEIGHT_OF_SPEAKER;
    }
    else if(whichTarget == 1){
      height = HEIGHT_OF_SPEAKER;
    }
    else if(whichTarget == 2){
      height = HEIGHT_OF_AMP;
    }
    else if(whichTarget == 3){
      height = HEIGHT_OF_SPEAKER;
    }
    else{
      height = HEIGHT_OF_STAGE;
    }
    double d = (height - HEIGHT_OF_CAMERA) / Math.tan(Math.toRadians(CAMERA_MOUNT_ANGLE + y));
    return d-6;       
  }

}