// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final double SHOOTER_RPM_MAX = 5676; ///60 to make rpm to rps,  TODO: get actual max RPM
  private final double SHOOTER_FEEDFORWARD_KS = 0.05; //probably the same as before
  private final double SHOOTER_FEEDFORWARD_KV = 12.0 / SHOOTER_RPM_MAX;

  private CANSparkMax flywheelTopLeft;
  private CANSparkMax flywheelTopFollower;
  private CANSparkMax flywheelBottomLeft;
  private CANSparkMax flywheelBottomFollower;
  private CANSparkMax loader;
  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;

  private PIDController topControl;
  private PIDController bottomControl;
  private SimpleMotorFeedforward ff;

  private boolean manualOverride;

  private DigitalInput primaryNoteDetect;
  private DigitalInput backupNoteDetect;

  /** Creates a new Shooter. */
  public Shooter() {
    primaryNoteDetect = new DigitalInput(0);
    backupNoteDetect = new DigitalInput(1);


    flywheelTopLeft = new CANSparkMax(40, MotorType.kBrushless);
    flywheelTopLeft.setInverted(false);
    flywheelTopFollower = new CANSparkMax(41, MotorType.kBrushless);
    flywheelTopFollower.follow(flywheelTopLeft , !flywheelTopLeft.getInverted());
    topEncoder = flywheelTopLeft.getEncoder();
    topControl = new PIDController(1, 0, 0);

    flywheelBottomLeft = new CANSparkMax(42, MotorType.kBrushless);
    flywheelBottomLeft.setInverted(true);
    flywheelBottomFollower = new CANSparkMax(43, MotorType.kBrushless);
    flywheelBottomFollower.follow(flywheelBottomLeft , flywheelBottomLeft.getInverted());
    bottomEncoder = flywheelBottomLeft.getEncoder();
    bottomControl = new PIDController(1, 0, 0);

    ff = new SimpleMotorFeedforward(SHOOTER_FEEDFORWARD_KS, SHOOTER_FEEDFORWARD_KV);

    loader = new CANSparkMax(44, MotorType.kBrushless);

    manualOverride = true;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Top Flywheel", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Flywheel", bottomEncoder.getVelocity());
    // This method will be called once per scheduler run
  }

  public void loadNote(){
    loader.set(1);
  }
  public void holdNote(){
    loader.set(0);
  }
  public void backdriveNote(){
    loader.set(-1);
  }

  public void fireNoteManual(double speed , double load){
    speed *= .5;
    if(Math.abs(speed) > .1){
      flywheelTopLeft.setVoltage(speed * 12);
      flywheelBottomLeft.setVoltage(speed * 12);
    }
    else{
      flywheelTopLeft.set(0);
      flywheelBottomLeft.set(0);
    }
    runLoader(load);
  }

  public void fireNote(int rpm , double load){
    double setpoint = rpm;
    flywheelTopLeft.setVoltage(topControl.calculate(topEncoder.getVelocity(), setpoint) + ff.calculate(setpoint));
    flywheelBottomLeft.setVoltage(bottomControl.calculate(bottomEncoder.getVelocity() , setpoint) + ff.calculate(setpoint));
    runLoader(load);

  }

  private void runLoader(double load){
    if(Math.abs(load) > .1){
      loader.set(load);
    }
    else{
      loader.set(0);
    }

  }

  public void stopShooter(){
    flywheelTopLeft.set(0);
    flywheelBottomLeft.set(0);
  }

  public boolean getNoteDetect(){
    return primaryNoteDetect.get() || backupNoteDetect.get();
  }


}
