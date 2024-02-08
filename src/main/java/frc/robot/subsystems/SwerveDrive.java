// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private final double L = 22 + (3 / 8.0);//in inches must be measured axel to axel
  private final double W = 22 + (3 / 8.0);
  private double r;

  private SwervePod backLeft;
  private SwervePod frontLeft;
  private SwervePod frontRight;
  private SwervePod backRight;

  private PigeonIMU imu;
  private PigeonIMU.GeneralStatus imuStatus;
  private int imuErrorCode;
  private double[] ypr;

  private boolean turbo = true;

  private boolean fieldOriented = true;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    imu = new PigeonIMU(30);
    imuStatus = new PigeonIMU.GeneralStatus();
    ypr = new double[3];
    imu.setYaw(0);

    backLeft = new SwervePod(4);
    frontLeft = new SwervePod(3);
    frontRight = new SwervePod(2);
    backRight = new SwervePod(5);

    r = Math.sqrt((L * L) + (W * W));
    
    //turbo = false;

  }

  public void turboOn(){
    turbo = true;
  }
  public void turboOff(){
    turbo = false;
  }

  public void resetYaw(){
    imu.setYaw(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    imuErrorCode = imu.getGeneralStatus(imuStatus).value;
    imu.getYawPitchRoll(ypr);
    SmartDashboard.putNumber("IMU Health", imuErrorCode);
    SmartDashboard.putNumber("IMU Yaw", ypr[0]);
  }

  public void podDriver(double x1 , double y1 , double x2){
  
    if(Math.abs(x1) <= .1){
      x1 = 0;
    }
    if(Math.abs(y1) <= .1){
      y1 = 0;
    }
    if(Math.abs(x2) <= .1){
      x2 = 0;
    }

    if(!turbo){
      y1 *= .5;
    }

    //field oriented drive controller mod option, set to true in instance variables
    if(fieldOriented){
      double yawRad = ypr[0] * Math.PI / 180;
      double temp = y1 * Math.cos(yawRad) + x1 * Math.sin(yawRad);
      x1 = -y1 * Math.sin(yawRad) + x1 * Math.cos(yawRad);//navx makes y1 negative because clockwise is positive, pigeon is counter clockwise positive I think
      y1 = temp;
    }

    double a = x1 - x2 * (L / r);
    double b = x1 + x2 * (L / r);
    double c = y1 - x2 * (W / r);
    double d = y1 + x2 * (W / r);

    double backRightSpeed = Math.sqrt ((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

    double backRightAngle = (Math.atan2 (a, d) / Math.PI) * 180;
    double backLeftAngle = (Math.atan2 (a, c) / Math.PI) * 180;
    double frontRightAngle = (Math.atan2 (b, d) / Math.PI) * 180;
    double frontLeftAngle = (Math.atan2 (b, c) / Math.PI) * 180;

    /*SmartDashboard.putNumber("BR Speed", backRightSpeed);
    SmartDashboard.putNumber("BR Angle", backRightAngle);
    SmartDashboard.putNumber("BL Speed", backLeftSpeed);
    SmartDashboard.putNumber("BL Angle", backLeftAngle);
    SmartDashboard.putNumber("FR Speed", frontRightSpeed);
    SmartDashboard.putNumber("FR Angle", frontRightAngle);
    SmartDashboard.putNumber("FL Speed", frontLeftSpeed);
    SmartDashboard.putNumber("FL Angle", frontLeftAngle);
    SmartDashboard.putNumber("x1", x1);
    SmartDashboard.putNumber("y1", y1);
    SmartDashboard.putNumber("x2", x2);*/

    /*backRight.drivePod (backRightSpeed, backRightAngle, ypr[0]);
    backLeft.drivePod (backLeftSpeed, backLeftAngle, ypr[0]);
    frontRight.drivePod (frontRightSpeed, frontRightAngle, ypr[0]);
    frontLeft.drivePod (frontLeftSpeed, frontLeftAngle, ypr[0]);*/

    backRight.drivePod (backRightSpeed, backRightAngle);
    backLeft.drivePod (backLeftSpeed, backLeftAngle);
    frontRight.drivePod (frontRightSpeed, frontRightAngle);
    frontLeft.drivePod (frontLeftSpeed, frontLeftAngle);

  }

  public void stop(){
    podDriver(0, 0, 0);

  }

  /*public void setDirection(double direction){
    pod1.setDirection(direction);
  }*/
}
