// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final double SHOOTER_RPM_MAX = 5800; ///60 to make rpm to rps,  TODO: get actual max RPM
  private final double SHOOTER_FEEDFORWARD_KS = 0.05; //probably the same as before
  private final double SHOOTER_FEEDFORWARD_KV = 12.0 / SHOOTER_RPM_MAX;

  private CANSparkMax flywheelTopLeft;
  private CANSparkMax flywheelTopFollower;
  private CANSparkMax flywheelBottomLeft;
  private CANSparkMax flywheelBottomFollower;
  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;

  private PIDController topControl;
  private PIDController bottomControl;
  private SimpleMotorFeedforward ff;

  /** Creates a new Shooter. */
  public Shooter() {


    flywheelTopLeft = new CANSparkMax(40, MotorType.kBrushless);
    flywheelTopLeft.setInverted(true);
    flywheelTopFollower = new CANSparkMax(41, MotorType.kBrushless);
    flywheelTopFollower.follow(flywheelTopLeft , true);
    topEncoder = flywheelTopLeft.getEncoder();
    topControl = new PIDController(.001, 0, 0);

    flywheelBottomLeft = new CANSparkMax(42, MotorType.kBrushless);
    flywheelBottomLeft.setInverted(false);
    flywheelBottomFollower = new CANSparkMax(43, MotorType.kBrushless);
    flywheelBottomFollower.follow(flywheelBottomLeft , true);
    bottomEncoder = flywheelBottomLeft.getEncoder();
    bottomControl = new PIDController(.002, 0, 0);

    ff = new SimpleMotorFeedforward(SHOOTER_FEEDFORWARD_KS, SHOOTER_FEEDFORWARD_KV);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Top Flywheel", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Flywheel", bottomEncoder.getVelocity());
    // This method will be called once per scheduler run
  }


  public void fireNoteManual(double speed){
    speed *= .5;
    if(Math.abs(speed) > .1){
      flywheelTopLeft.setVoltage(speed * 2);
      flywheelBottomLeft.setVoltage(speed * 12);
    }
    else{
      flywheelTopLeft.set(0);
      flywheelBottomLeft.set(0);
    }
 }

  public void fireNote(double upper , double lower){
    double setpoint = upper;//<30  2000 4000
    double setpoint2 = lower;//30-36 2000 2000
    flywheelTopLeft.setVoltage(topControl.calculate(topEncoder.getVelocity(), setpoint) + ff.calculate(setpoint));
    flywheelBottomLeft.setVoltage(bottomControl.calculate(bottomEncoder.getVelocity() , setpoint2) + ff.calculate(setpoint2));
  }
  public void fireNote(){
    double setpoint = 2000;//when all else fails 2000 4000 against the sub
    double setpoint2 = 4000;
    flywheelTopLeft.setVoltage(topControl.calculate(topEncoder.getVelocity(), setpoint) + ff.calculate(setpoint));
    flywheelBottomLeft.setVoltage(bottomControl.calculate(bottomEncoder.getVelocity() , setpoint2) + ff.calculate(setpoint2));
  }

  public void fireNoteWall(){//in case of wall
    //double setpoint = 2000;//old shooter
    //double setpoint2 = 2000;
    double setpoint = 4700;//new shooter
    double setpoint2 = 1100;
    flywheelTopLeft.setVoltage(topControl.calculate(topEncoder.getVelocity(), setpoint) + ff.calculate(setpoint));
    flywheelBottomLeft.setVoltage(bottomControl.calculate(bottomEncoder.getVelocity() , setpoint2) + ff.calculate(setpoint2));

  }

  public void fireNoteAmp() {
    double ampMod = .2;
    double setPoint = 1500;
    flywheelTopLeft.setVoltage(topControl.calculate(topEncoder.getVelocity(), setPoint * ampMod) + ff.calculate(setPoint * ampMod));
    flywheelBottomLeft.setVoltage((bottomControl.calculate(bottomEncoder.getVelocity(), setPoint) + ff.calculate(setPoint)));

  }

  public void stopShooter(){
    flywheelTopLeft.set(0);
    flywheelBottomLeft.set(0);
  }

}
