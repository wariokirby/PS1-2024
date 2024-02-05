// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  private CANSparkMax deploy;
  private CANSparkMax collect;
  private RelativeEncoder deployEncoder;
  private final double DOWN_POSITION = .25;//TODO change to how many rotations it actually takes to deploy
  private final double UP_POSITION = .1;//TODO tune this so the motor is not trying to get to a position it can't reach mechanically
  private boolean down;

  /** Creates a new Collector. */
  public Collector() {
    deploy = new CANSparkMax(43, MotorType.kBrushless);
    deployEncoder = deploy.getEncoder();
    deployEncoder.setPositionConversionFactor(1 / 100);
    collect = new CANSparkMax(44, MotorType.kBrushless);
    down = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deployCollector(){
    down = true;
  }

  public void retractCollector(){
    down = false;
  }

  public void runCollector(double speed){
    if(down && deployEncoder.getPosition() < DOWN_POSITION){
      deploy.set(DOWN_POSITION - deployEncoder.getPosition() / DOWN_POSITION);//TODO Down may need a speed limiter
    }
    else if(!down && deployEncoder.getPosition() > UP_POSITION){
      deploy.set(UP_POSITION - deployEncoder.getPosition() / DOWN_POSITION);//may need more power
    }
    else{
      deploy.set(0);
    }//end position holder

    if(Math.abs(speed) > .1){
      collect.set(speed);
    }
    else{
      collect.set(0);
    }
  }
}
