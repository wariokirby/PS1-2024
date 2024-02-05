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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final double SHOOTER_RPS_MAX = 5676 / 60.0; //stated rpm to rps,  TODO: get actual max RPS
  private final double SHOOTER_FEEDFORWARD_KS = 0.05; //probably the same as before
  private final double SHOOTER_FEEDFORWARD_KV = 12.0 / SHOOTER_RPS_MAX;

  private CANSparkMax flywheelTop;
  private CANSparkMax flywheelBottom;
  private CANSparkMax loader;
  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;

  private PIDController topControl;
  private PIDController bottomControl;
  private SimpleMotorFeedforward ff;

  private boolean manualOverride;

  private DigitalInput noteDetect;

  /** Creates a new Shooter. */
  public Shooter() {
    noteDetect = new DigitalInput(0);

    flywheelTop = new CANSparkMax(40, MotorType.kBrushless);
    flywheelTop.setInverted(false);
    topEncoder = flywheelTop.getEncoder();
    topControl = new PIDController(1, 0, 0);

    flywheelBottom = new CANSparkMax(41, MotorType.kBrushless);
    flywheelBottom.setInverted(true);
    bottomEncoder = flywheelBottom.getEncoder();
    bottomControl = new PIDController(1, 0, 0);

    ff = new SimpleMotorFeedforward(SHOOTER_FEEDFORWARD_KS, SHOOTER_FEEDFORWARD_KV);

    loader = new CANSparkMax(42, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
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

  public void fireNote(double speed){
    if(manualOverride){
      if(Math.abs(speed) > .1){
        flywheelTop.set(speed);
        flywheelBottom.set(speed);
      }
      else{
        flywheelTop.set(0);
        flywheelBottom.set(0);
      }
    }
    else{
      double setpoint = SHOOTER_RPS_MAX * speed;
      flywheelTop.setVoltage(topControl.calculate(topEncoder.getVelocity(), setpoint) + ff.calculate(setpoint));
      flywheelBottom.setVoltage(bottomControl.calculate(bottomEncoder.getVelocity() , setpoint) + ff.calculate(setpoint));
    }
  }

  public boolean getNoteDetect(){
    return noteDetect.get();
  }


}
