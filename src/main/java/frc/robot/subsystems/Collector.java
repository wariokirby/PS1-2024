// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Collector extends SubsystemBase {
  private CANSparkMax deploy;
  private CANSparkMax collect;

  private RelativeEncoder deployEncoder;
  private final double DOWN_POSITION = -.38;//TODO change to how many rotations it actually takes to deploy
  private final double UP_POSITION = -.05;//TODO tune this so the motor is not trying to get to a position it can't reach mechanically
  private boolean override;

  private DigitalInput primaryNoteDetect;
  private int RUMBLE_TIMER = 25;

  private CommandXboxController xboxD;
  private CommandXboxController xboxO;


  /** Creates a new Collector. */
  public Collector(CommandXboxController xboxD , CommandXboxController xboxO) {
    deploy = new CANSparkMax(44, MotorType.kBrushless);
    deployEncoder = deploy.getEncoder();
    deploy.setInverted(true);
    deployEncoder.setPositionConversionFactor(1 / 100.0);
    collect = new CANSparkMax(45, MotorType.kBrushless);

    override = false;

    primaryNoteDetect = new DigitalInput(0);
    this.xboxD = xboxD;
    this.xboxO = xboxO;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Collector Position", deployEncoder.getPosition());
    SmartDashboard.putBoolean("Have Note" , !primaryNoteDetect.get());
    //if()
    //xboxD.getHID().setRumble(RumbleType.kBothRumble , 1);
    // This method will be called once per scheduler run
  }
  public void enableOverride(){
    override = true;
  }
  public void disableOverride(){
    override = false;
    deployEncoder.setPosition(0);
  }

  public void manual(double speed , double dSpeed){
    if(Math.abs(speed) > .1){
      collect.set(speed);
    }
    else{
      collect.set(0);
    }
    if(override){
      deploy.set(dSpeed * .375);
    }
    else if(Math.abs(dSpeed) > .1){
      if(isUp() && dSpeed > 0){
        dSpeed = 0;
      }
      if(isDown() && dSpeed < 0){
        dSpeed = 0;
      }
      deploy.set(dSpeed * .375);
    }
    else{
      deploy.set(0);
    }
  }

  

  public boolean isUp(){
    return deployEncoder.getPosition() > UP_POSITION;
  }

  public boolean isDown(){
    return deployEncoder.getPosition() < DOWN_POSITION;
  }

  /*public void holdCollector(boolean lower , boolean raise){
    if(raise){
      down = false;
    }
    else if(lower){
      down = true;
    }

    if(down && deployEncoder.getPosition() < DOWN_POSITION){
      if(deployEncoder.getPosition() < .3){
        deploy.set(1);//run full until the tipping point //TODO tune where to start cutting power      
      }
      else{
        deploy.set(.15);//end slow
      }
    }
    else if(!down && deployEncoder.getPosition() > UP_POSITION){
      if(deployEncoder.getPosition() > .2){
        deploy.set(-1);//run full until the tipping point //TODO tune where to start cutting power
      }
      else{
        deploy.set(-.15);//end slow
      }
    }
    else{
      deploy.set(0);
    }//end position holder
  }*/

  public void intake(){
    collect.set(-.5);
  }

  public void fire(){
    collect.set(1);//1
  }

  public void fireAmp(){
    collect.set(.3);
  }

  public boolean getNoteDetect(){
    return primaryNoteDetect.get();
  }

  public void off(){
    collect.set(0);
    deploy.set(0);
  }


}
