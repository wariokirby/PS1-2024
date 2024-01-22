// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private final double L = 22 + (3 / 16);//in inches must be measured axel to axel
  private final double W = 21 + (13 / 16);
  private double r;

  private final double POD1_OFFSET = -199;
  private final double POD2_OFFSET = -8;
  private final double POD3_OFFSET = -43;
  private final double POD4_OFFSET = -19;


  private SwervePod backLeft;//pod1
  private SwervePod frontLeft;//pod2
  private SwervePod frontRight;//pod3
  private SwervePod backRight;//pod4

  private PigeonIMU imu;
  private PigeonIMU.GeneralStatus imuStatus;
  private int imuErrorCode;
  private double[] ypr;

  private int whichPod;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    var pigTalon = new WPI_TalonSRX(20);
    imu = new PigeonIMU(pigTalon);
    imuStatus = new PigeonIMU.GeneralStatus();
    ypr = new double[3];
    imu.setYaw(0);

    backLeft = new SwervePod(1 , POD1_OFFSET , false);
    frontLeft = new SwervePod(2 , POD2_OFFSET , false);
    frontRight = new SwervePod(3 , POD3_OFFSET , false);
    backRight = new SwervePod(4 , POD4_OFFSET , false);

    whichPod = 1;

    r = Math.sqrt((L * L) + (W * W));
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    imuErrorCode = imu.getGeneralStatus(imuStatus).value;
    imu.getYawPitchRoll(ypr);
    SmartDashboard.putNumber("IMU Health", imuErrorCode);
    SmartDashboard.putNumber("IMU Yaw", ypr[0]);
  }

  public SwervePod getPod(int pod){
    switch(pod){
      case 1:
        return backLeft;
      case 2:
        return frontLeft;
      case 3:
        return frontRight;
       case 4:
        return backRight;
      default:
        return null;
      }
  }

  public void podTester(double drive , double turn){
    switch(whichPod){
      case 1:
        backLeft.spinWheel(drive);
        backLeft.turnPod(turn);
        break;
      case 2:
        frontLeft.spinWheel(drive);
        frontLeft.turnPod(turn);
        break;
      case 3:
        frontRight.spinWheel(drive);
        frontRight.turnPod(turn);
        break;
      case 4:
        backRight.spinWheel(drive);
        backRight.turnPod(turn);
        break;
    }//end switch
  }

  public void switcher(int whichPod){
    this.whichPod = whichPod;
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

    SmartDashboard.putNumber("BR Speed", backRightSpeed);
    SmartDashboard.putNumber("BR Angle", backRightAngle);
    SmartDashboard.putNumber("BL Speed", backLeftSpeed);
    SmartDashboard.putNumber("BL Angle", backLeftAngle);
    SmartDashboard.putNumber("FR Speed", frontRightSpeed);
    SmartDashboard.putNumber("FR Angle", frontRightAngle);
    SmartDashboard.putNumber("FL Speed", frontLeftSpeed);
    SmartDashboard.putNumber("FL Angle", frontLeftAngle);
    SmartDashboard.putNumber("x1", x1);
    SmartDashboard.putNumber("y1", y1);
    SmartDashboard.putNumber("x2", x2);

    backRight.drivePod (backRightSpeed, backRightAngle, ypr[0]);
    backLeft.drivePod (backLeftSpeed, backLeftAngle, ypr[0]);
    frontRight.drivePod (frontRightSpeed, frontRightAngle, ypr[0]);
    frontLeft.drivePod (frontLeftSpeed, frontLeftAngle, ypr[0]);

  }

  public void stop(){
    podDriver(0, 0, 0);

  }

  /*public void setDirection(double direction){
    pod1.setDirection(direction);
  }*/
}
