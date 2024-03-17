// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FinderL extends SubsystemBase {  
  private NetworkTable pixySub;
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  private double validTarget;
  private double x;
  private double y;

  /** Creates a new FinderL. */
  public FinderL() {
    pixySub = NetworkTableInstance.getDefault().getTable("limelight-pixysub");
    tv = pixySub.getEntry("tv");
    tx = pixySub.getEntry("tx");
    ty = pixySub.getEntry("ty");
    ta = pixySub.getEntry("ta");
    pixySub.getEntry("pipeline").setNumber(0);
    pixySub.getEntry("camMode").setNumber(0);



    //the following are out of range indicating no target found
    validTarget = 0;
    x = 200;
    y = 200;

  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Pixy Status", "Fired");

    validTarget = tv.getDouble(0);
    x = 3.8 * tx.getDouble(0);
    y = 7.7 * ty.getDouble(0);

    SmartDashboard.putBoolean("Notes found", validTarget > 0);
    SmartDashboard.putNumber("largest y location", x);
    SmartDashboard.putNumber("largest x location", y);

    // This method will be called once per scheduler run
  }

  public double[] findClosestTarget(){
    double[] directionSize = new double[3];//0 is amount from center of view from -157 to 157, 1 is amount from center vertically from -103 to 103 positive down, 2 is width of the target
    if(validTarget == 0){
      directionSize[0] = 200;//set outside range to know the target is not in view
      directionSize[1] = 200;
      directionSize[2] = -1;//set outside range to know the target is not in view
      return directionSize;
    }
    directionSize[0]=-ty.getDouble(0);//setting the center of the camera view to be center. sideways so -y is now x 
    directionSize[1]=tx.getDouble(0);//setting the center of the camera view to be center.  
    directionSize[2]=ta.getDouble(0);
    return directionSize;
  }//end findClosestTarget

}