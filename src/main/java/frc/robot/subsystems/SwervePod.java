// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwervePod extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkFlex swerveMotor;
  private CANcoder dirEnc;
  private RelativeEncoder driveEnc;
  private PIDController directionControl;

  //private double fieldAdjust;

  private int podID;

  private final double S_P = 1.0/150;
  private final double S_I = 0;
  private final double S_D = 0;

  /** Creates a new SwervePod. */
  public SwervePod(int driveID) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    swerveMotor = new CANSparkFlex(driveID + 10, MotorType.kBrushless);
    //fieldAdjust = 0;
    dirEnc = new CANcoder(driveID + 20);

    driveEnc = driveMotor.getEncoder();

    directionControl = new PIDController(S_P, S_I, S_D);
    directionControl.enableContinuousInput(-180, 180);

    driveEnc.setPositionConversionFactor(((4/12.0) * Math.PI) / (8.14)); //circumference for 4" wheel divided by 12" to a foot / gear ratio * -> feet
    driveEnc.setVelocityConversionFactor(((4/12.0) * Math.PI) / (8.14 * 60));//circumference for 4" wheel divided by 12" to a foot / gear ratio * convert to seconds -> feet per second

    podID = driveID;
  }

 public SwervePod(int driveID, int spareID) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    swerveMotor = new CANSparkFlex(spareID + 10, MotorType.kBrushless);
    //fieldAdjust = 0;
    dirEnc = new CANcoder(spareID + 20);

    driveEnc = driveMotor.getEncoder();

    directionControl = new PIDController(S_P, S_I, S_D);
    directionControl.enableContinuousInput(-180, 180);

    driveEnc.setPositionConversionFactor(((4/12.0) * Math.PI) / (8.14)); //circumference for 4" wheel divided by 12" to a foot / gear ratio * -> feet
    driveEnc.setVelocityConversionFactor(((4/12.0) * Math.PI) / (8.14 * 60));//circumference for 4" wheel divided by 12" to a foot / gear ratio * convert to seconds -> feet per second

    podID = driveID;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Speed" + podID, getSpeed());
    SmartDashboard.putNumber("Distance" + podID, getDistance());
    SmartDashboard.putNumber("Angle" + podID, getAngle());
    // This method will be called once per scheduler run
  }

  public double getSpeed(){
    return driveEnc.getVelocity();
  }

  public double getDistance(){
    return driveEnc.getPosition();
  }

  public double getAngle(){
    //double angle = (dirEnc.getAbsolutePosition().getValue() * 360) + fieldAdjust;
    double angle = (dirEnc.getAbsolutePosition().getValue() * 360);
    /*if(angle > 180){
      angle -= 360;
    }
    if(angle < -180){
      angle += 360;
    }*/
    return angle;
  }

  public void turnPod(double turn){
      swerveMotor.set(turn);
  }

  public void setDirection(double direction){
    double altDir;
    if(direction > 0){
      altDir = direction - 180;
    }
    else{
      altDir = direction + 180;
    }
    
    if(Math.abs(getAngle() - direction) > Math.abs(getAngle() - altDir)){
      direction = altDir;
      driveMotor.setInverted(true);
    }
    else{
      driveMotor.setInverted(false);
    }
    SmartDashboard.putNumber("pid" + podID, directionControl.calculate(getAngle(), direction));
    turnPod(directionControl.calculate(getAngle(), direction));
  }

  public void drivePod(double drive , double direction){
    //fieldAdjust = yaw;//add a yaw parameter and set it up in SwerveDrive class if switching back to changing the encoder offset for field oriented
    setDirection(direction);
    if(Math.abs(drive) > .1){
      driveMotor.set(drive);
    }
    else{
      driveMotor.set(0);
    }
  }

  
}
