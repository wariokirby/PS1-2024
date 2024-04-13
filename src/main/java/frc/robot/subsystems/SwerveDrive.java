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


  private boolean fieldOriented = true;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    imu = new PigeonIMU(30);
    imuStatus = new PigeonIMU.GeneralStatus();
    ypr = new double[3];
    imu.setYaw(0);

    backLeft = new SwervePod(2 , 7 , false);
    backRight = new SwervePod(3 , 8 , false);
    frontLeft = new SwervePod(5 , 5 , false);
    frontRight = new SwervePod(4 , 4 , false);

    r = Math.sqrt((L * L) + (W * W));
    
    fieldOriented = true;
  }

  public void turboOn(){
    backLeft.turbo();
    backRight.turbo();
    frontLeft.turbo();
    frontRight.turbo();
  }
  public void turboOff(){
    backLeft.nerf();
    backRight.nerf();
    frontLeft.nerf();
    frontRight.nerf();
  }

  public void enableFieldOriented(){
    fieldOriented = true;
  }
  public void disableFieldOriented(){
    fieldOriented = false;
  }

  public void resetYaw(){
    imu.setYaw(0);
  }
  public double getYaw(){
    return ypr[0];
  }

  public void resetDistances(boolean all){//false does not reset front right to have a total distance traveled
    backLeft.resetDistance();
    backRight.resetDistance();
    frontLeft.resetDistance();
    if(all){
      frontRight.resetDistance();
    }
    
  }
  public double getAverageDistance(){
    return (backLeft.getDistance() + backRight.getDistance() + frontLeft.getDistance()) / 3.0;
  }
  public double getAbsoluteDistance(){
    return frontRight.getDistance();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    imuErrorCode = imu.getGeneralStatus(imuStatus).value;
    imu.getYawPitchRoll(ypr);
    SmartDashboard.putBoolean("IMU Health", imuErrorCode == 0);
    SmartDashboard.putNumber("IMU Yaw", ypr[0]);
    SmartDashboard.putNumber("AVG Distance", getAverageDistance() * 12);
  }

  public void podDriver(double x1 , double y1 , double x2 , boolean turnDZ , boolean driveDZ){
  
    if(Math.abs(x1) <= .1 && driveDZ){
      x1 = 0;
    }
    if(Math.abs(y1) <= .1 && driveDZ){
      y1 = 0;
    }
    if((Math.abs(x2) <= .1 && turnDZ)){
      x2 = 0;
    }


    if(fieldOriented){
      double yawRad = ypr[0] * Math.PI / 180;
      double temp = y1 * Math.cos(yawRad) + x1 * Math.sin(yawRad);
      x1 = -y1 * Math.sin(yawRad) + x1 * Math.cos(yawRad);
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

    backRight.drivePod (backRightSpeed, backRightAngle);
    backLeft.drivePod (backLeftSpeed, backLeftAngle);
    frontRight.drivePod (frontRightSpeed, frontRightAngle);
    frontLeft.drivePod (frontLeftSpeed, frontLeftAngle);

  }

  public void stop(){
    podDriver(0, 0, 0 , false , false);

  }

}
