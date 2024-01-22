// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwervePod extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkMax swerveMotor;
  private CANCoder dirEnc;
  private RelativeEncoder driveEnc;
  private PIDController directionControl;

  private final int SPARE_POD_ENCODER_ID = 25;
  private final double SPARE_POD_ENCODER_OFFSET = 0;//TODO get real offset

  private double encoderOffset;
  private double fieldAdjust;

  private int podID;
  //private boolean usingSpare;
  /** Creates a new SwervePod. */
  public SwervePod(int driveID , double encoderOffset , boolean usingSpare) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    swerveMotor = new CANSparkMax(driveID + 10, MotorType.kBrushless);
    this.encoderOffset = encoderOffset;
    fieldAdjust = 0;

    if(usingSpare){
      dirEnc = new CANCoder(SPARE_POD_ENCODER_ID);
      encoderOffset = SPARE_POD_ENCODER_OFFSET;
    }
    else{
      dirEnc = new CANCoder(driveID + 20);
    }

    driveEnc = driveMotor.getEncoder();

    directionControl = new PIDController((1.0/150), 0, 0);
    directionControl.enableContinuousInput(-180, 180);

    //uncomment when switching to real units instead of rotations and rpm
    //velEnc.setPositionConversionFactor(((4/12) * Math.PI) / (8.14)); //circumference for 4" wheel divided by 12" to a foot / gear ratio * -> feet
    //velEnc.setVelocityConversionFactor(((4/12) * Math.PI) / (8.14 * 60));//circumference for 4" wheel divided by 12" to a foot / gear ratio * convert to seconds -> feet per second

    podID = driveID;
    //this.usingSpare = usingSpare;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Speed" + podID, getSpeed());
    SmartDashboard.putNumber("Distance" + podID, getDistance());
    SmartDashboard.putNumber("Angle" + podID, getAngle());
    // This method will be called once per scheduler run
  }

  public double getSpeed(){
    return driveEnc.getVelocity() / 8.14;
  }

  public double getDistance(){
    return driveEnc.getPosition() / 8.14;
  }

  public double getAngle(){
    double angle = dirEnc.getAbsolutePosition() + encoderOffset + fieldAdjust;
    if(angle > 180){
      angle -= 360;
    }
    if(angle < -180){
      angle += 360;
    }

    return angle;
  }

  public void turnPod(double turn){
    //if(Math.abs(turn) > .1){
      swerveMotor.set(turn);
    /*}
    else{
      swerveMotor.set(0);
    }*/
    
  }

  public void spinWheel(double speed){
    if(Math.abs(speed) > .5){
      driveMotor.set(speed);
      }
    else{
      swerveMotor.set(0);
    }
}

  /*public void setDirection(double direction){
    if(direction > 90){
      direction -= 180;
      driveMotor.setInverted(true);
    }
    else if(direction < -90){
      direction += 180;
      driveMotor.setInverted(true);
    }
    else{
      driveMotor.setInverted(false);
    }
    SmartDashboard.putNumber("pid" + podID, directionControl.calculate(getAngle(), direction));
    turnPod(directionControl.calculate(getAngle(), direction));
  }*/


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

  public void drivePod(double drive , double direction , double yaw){
    fieldAdjust = yaw;
    setDirection(direction);
    if(Math.abs(drive) > .1){
      driveMotor.set(drive);
    }
    else{
      driveMotor.set(0);
    }
  }

  
}
