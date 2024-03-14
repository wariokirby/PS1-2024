// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This class is an example of how to find the closest (or largest) item with the programmed signature
//the display will show how many targets are in view, how far off from center the nearest target is as well as the width of the nearest target
//This is also based on PseudoResonance's repo, coming from the examples

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.pixy2api.Pixy2;
import frc.robot.pixy2api.Pixy2CCC;
import frc.robot.pixy2api.Pixy2CCC.Block;
import frc.robot.pixy2api.links.I2CLink;

public class Finder extends SubsystemBase {
  /** Creates a new Finder. */
  private Pixy2 pixy;
  private int targetCount;
  private int sig = Pixy2CCC.CCC_SIG1;
  private ArrayList<Block> targets;

  public Finder() {
    pixy = Pixy2.createInstance(new I2CLink());
    pixy.init(1);
    //pixy.setLamp((byte) 1, (byte) 1);
    targets = new ArrayList<Block>();
  }

  @Override
  public void periodic() {
    targetCount = pixy.getCCC().getBlocks(false, sig, 11);
    SmartDashboard.putNumber("Notes found", targetCount);

    //debugging code
    targets = pixy.getCCC().getBlockCache();
    int[] directionSize = findClosestTarget();
    SmartDashboard.putNumber("largest x location", directionSize[0] - 74);
    SmartDashboard.putNumber("largest y location", directionSize[1] + 24);
    SmartDashboard.putNumber("largest width", directionSize[2]);
    // This method will be called once per scheduler run
  }

  public int[] findClosestTarget(){
    int[] directionSize = new int[3];//0 is amount from center of view from -157 to 157, 1 is amount from center vertically from -103 to 103 positive down, 2 is width of the target
    if(targetCount == 0 || targets.size() == 0){
      directionSize[0] = 200;//set outside range to know the target is not in view
      directionSize[1] = 200;
      return directionSize;
    }
    if(targets.size() == 0){
      directionSize[0] = -1;//set outside range to know the target is not in view
      return directionSize;
    }
    //searching for the largest target in view which should be the closest
    Block closestTarget = targets.get(0);
    for(int index = 1; index<targets.size(); index++){
      if(targets.get(index).getWidth()>closestTarget.getWidth()){
        closestTarget = targets.get(index);
      }//end if
    }//end for
    directionSize[0]=closestTarget.getX()-157;//setting the center of the camera view to be center.  
    directionSize[1]=-1*(closestTarget.getY()-103);//setting the center of the camera view to be center.  
    directionSize[2]=closestTarget.getWidth();
    return directionSize;
  }//end findClosestTarget

}//end subsystem
