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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwervePod extends SubsystemBase {

  private CANSparkMax driveMotor;
  private CANSparkFlex swerveMotor;
  private CANcoder dirEnc;
  private RelativeEncoder driveEnc;
  //private double fieldAdjust;
  private int positionID;

  private PIDController directionControl;

  private final double S_P = 1.0/170;
  private final double S_I = 0;
  private final double S_D = 0;

  private PIDController velocityControl;
  //private ProfiledPIDController velocityControl;//attempting to reduce acceleration to manageable levels be sure to change both contructors when activating or disabling
  private final double SPEED_LIMIT = 11;//Actual top speed is about 11.7, limiting for vel control
  private SimpleMotorFeedforward ff;
  private boolean manualOverride;
 

  private final double D_P = .5;
  private final double D_I = 0;
  private final double D_D = 0;
  private final double KS = .05;
  private final double KV = 12.0 / 11.5;




  /** Creates a new SwervePod. */
  /*public SwervePod(int driveID) {
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

    manualOverride = false;
    //velocityControl = new PIDController(D_P, D_I, D_D);
    velocityControl = new ProfiledPIDController(D_P, D_I, D_D, new TrapezoidProfile.Constraints(SPEED_LIMIT , 20));
    ff = new SimpleMotorFeedforward(KS, KV);
  }*/

 public SwervePod(int positionID, int podID) {
    driveMotor = new CANSparkMax(positionID, MotorType.kBrushless);
    swerveMotor = new CANSparkFlex(podID + 10, MotorType.kBrushless);
    //fieldAdjust = 0;
    dirEnc = new CANcoder(podID + 20);

    driveEnc = driveMotor.getEncoder();

    directionControl = new PIDController(S_P, S_I, S_D);
    directionControl.enableContinuousInput(-180, 180);

    driveEnc.setPositionConversionFactor(((4/12.0) * Math.PI) / (8.14)); //circumference for 4" wheel divided by 12" to a foot / gear ratio * -> feet
    driveEnc.setVelocityConversionFactor(((4/12.0) * Math.PI) / (8.14 * 60));//circumference for 4" wheel divided by 12" to a foot / gear ratio * convert to seconds -> feet per second

    this.positionID = positionID;

    manualOverride = false;
    velocityControl = new PIDController(D_P, D_I, D_D);
    //velocityControl = new ProfiledPIDController(D_P, D_I, D_D, new TrapezoidProfile.Constraints(SPEED_LIMIT , 20));
    ff = new SimpleMotorFeedforward(KS, KV);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Speed" + positionID, getSpeed());
    SmartDashboard.putNumber("Distance" + positionID, getDistance());
    SmartDashboard.putNumber("Angle" + positionID, getAngle());
    // This method will be called once per scheduler run
  }

  public double getSpeed(){
    return driveEnc.getVelocity();
  }

  public double getDistance(){
    return driveEnc.getPosition();
  }

  public double getAngle(){
    double angle = (dirEnc.getAbsolutePosition().getValue() * 360);
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
    
    if(optimize(direction)){
      direction = altDir;
      driveMotor.setInverted(true);
    }
    else{
      driveMotor.setInverted(false);
    }
    turnPod(directionControl.calculate(getAngle(), direction));
  }

  private boolean optimize(double direction){
    double currAngle = getAngle();
    double altDir;
    if(direction > 0){
      altDir = direction - 180;
    }
    else{
      altDir = direction + 180;
    }
    double normWay = Math.abs(currAngle - direction);
    double altWay = Math.abs(currAngle - altDir);
    double normCross;
    double altCross;
    if(currAngle >= 0){
      normCross = (180 - currAngle) + (direction + 180);
      altCross = (180 - currAngle) + (altDir + 180);
    }
    else{
      normCross = (currAngle + 180) + (180 - direction);
      altCross = (currAngle + 180) + (180 - altDir);
    }
    return (altWay < normWay && altWay < normCross) || (altCross < normWay && altCross < normCross);

  }


  public void drivePod(double drive , double direction){
    //fieldAdjust = yaw;//add a yaw parameter and set it up in SwerveDrive class if switching back to changing the encoder offset for field oriented
    setDirection(direction);
    if(manualOverride){
      if(Math.abs(drive) > .1){
        driveMotor.set(drive);
      }
      else{
        driveMotor.set(0);
      }
    }//end no velocity control
    else{
      double setpoint = drive * SPEED_LIMIT;
      driveMotor.setVoltage(velocityControl.calculate(getSpeed(), setpoint) + ff.calculate(setpoint));
    }//end velocity control
  }

  public void setManualOverride(boolean manualOverride){
    this.manualOverride = manualOverride;
  }

  
}
