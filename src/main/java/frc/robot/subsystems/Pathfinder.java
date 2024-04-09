// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pathfinder extends SubsystemBase {
  //all dimensions are inches  104.64
  private double[] NOTE_1 = {0 , 115.955};//second note
  private double[] NOTE_2 = {57 , 115.955};//closer 3rd note or 4th note
  private double[] NOTE_3 = {-57 , 115.955};//this is the one touching the stage
  private double[] NOTE_4 = {75 , 326.555};//center area 3rd note
  private double[] NOTE_5 = {9 , 326.555};//hard to get center note with stage in the way
  private double ROBOT_CENTER = 17.5;//total length to the middle of the collector lower rollers 3 + 29 + 6.75;

  private SwerveDrive drive;
  private Targeting targeting;


  /** Creates a new Pathfinder. */
  public Pathfinder(SwerveDrive drive , Targeting targeting) {
    this.drive = drive;
    this.targeting = targeting;
  }

  @Override
  public void periodic() {
    double[] position = calcPosition();
    SmartDashboard.putNumber("x-coord", position[0]);
    SmartDashboard.putNumber("y-coord", position[1]);

    // This method will be called once per scheduler run
  }

  public void setSide(boolean isRed){
    if(isRed){
      NOTE_2[0] = 57;
      NOTE_3[0] = -57;
      NOTE_4[0] = 75;
      NOTE_5[0] = 9;
   }
   else{
      NOTE_2[0] = -57;
      NOTE_3[0] = 57;
      NOTE_4[0] = -75;
      NOTE_5[0] = -9;

   }
  }

  public double[] calcPosition(){//from robot center 
    double[] position = new double[2];//x-y
    double angle = Math.toRadians(drive.getYaw() + targeting.getX());
    double hyp = targeting.calcRange();
    if(targeting.getValidTarget() == 0){
      position[0] = 0;
      position[1] = 0;
    }
    else if(angle == 0){
      position[0] = 0;
      position[1] = hyp;
    }
    else{
      position[0] = hyp * Math.sin(angle);
      position[1] = hyp * Math.cos(angle) + ROBOT_CENTER;
    }
    return position;
  }

  public double[] plotCourse(int whichNote){//distance-angle
    double[] destination;
    double[] position = calcPosition();
    double[] course = new double[2];
    if(whichNote == 1){
      destination = NOTE_1;
    }
   else if(whichNote == 2){
      destination = NOTE_2;
    }
   else if(whichNote == 3){
      destination = NOTE_3;
    }
   else if(whichNote == 4){
      destination = NOTE_4;
    }
   else{
      destination = NOTE_5;
    }

    double x = destination[0] - position[0];
    double y = destination[1] - position[1];
    course[0] = Math.sqrt((x*x) + (y*y));
    course[1] = Math.toDegrees(Math.atan(x / y));
    return course;

  }
}
