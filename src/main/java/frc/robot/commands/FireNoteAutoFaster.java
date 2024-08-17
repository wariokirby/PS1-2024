// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Targeting;


public class FireNoteAutoFaster extends Command {
  private Shooter shooter;
  private Collector collector;
  private SwerveDrive drivetrain;
  private Targeting targeting;
  private boolean secondShot;
  private int timer;
  private boolean noTarget;
  /** Creates a new FireNote. */
  public FireNoteAutoFaster(Shooter shooter , Collector collector , SwerveDrive drivetrain , Targeting targeting , boolean secondShot) {
    this.shooter = shooter;
    this.collector = collector;
    this.drivetrain = drivetrain;
    this.targeting = targeting;
    this.secondShot = secondShot;
    noTarget = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter , collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collector.intake();
    targeting.changeTag(0);
    if(targeting.getValidTarget() == 0){
      targeting.changeTag(1);
      if(targeting.getValidTarget() == 0){
        noTarget = true;
      }
    }
    timer = 28;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    newShooter();

    timer--;
    if(timer <= 25){
      collector.fire();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer <= 0;
 }

  public void newShooter(){
    if(noTarget && !secondShot){
      shooter.fireNote(4700 , 1000);//range 81.75
    }
    else if(noTarget && Math.abs(drivetrain.getYaw()) > 5){
      shooter.fireNote(4700 , 900);//range 91.25 stage note
    }
    else if(noTarget || targeting.calcRange() > 91){
       shooter.fireNote(4700 , 1100);//range 77.08
    }
    else{
      drivetrain.podDriver(0, 0, -(targeting.getX()/60) , false , false);

      if(targeting.calcRange() <= 40){
        shooter.fireNote(2000 , 4000);
      }
      else if(targeting.calcRange() <= 44){
        shooter.fireNote(2000 , 3000);
      }
      else if(targeting.calcRange() <= 60){
        shooter.fireNote(2000 , 2000);
      }
      else if(targeting.calcRange() <= 64){
        shooter.fireNote(4000 , 2000);
      }
      else if(targeting.calcRange() <= 72){
        shooter.fireNote(4000 , 1500);
      }
      else if(targeting.calcRange() <= 74){
        shooter.fireNote(4700 , 1250);
      }
      else if(targeting.calcRange() <= 76){
        shooter.fireNote(4700 , 1100);
      }
      else if(targeting.calcRange() <= 84){
        shooter.fireNote(4700 , 1000);
      }
      else{
        shooter.fireNote(4700 , 900);
      }
    }
  }

  public void oldShooter(){
    if(noTarget && !secondShot){
      shooter.fireNote(4000 , 1750);//range 81.75
    }
    else if(noTarget){
      shooter.fireNote(4000 , 1500);//range 91.25 stage note
    }
    else{
      drivetrain.podDriver(0, 0, -(targeting.getX()/50) , false , false);

      if(targeting.calcRange() <= 45){
        shooter.fireNote(2000 , 4000);
      }
      else if(targeting.calcRange() <= 55){
        shooter.fireNote(2000 , 3000);
      }
      else if(targeting.calcRange() <= 74){
        shooter.fireNote(2000 , 2000);
      }
      else if(targeting.calcRange() <= 79){
        shooter.fireNote(4000 , 1900);
      }
      else if(targeting.calcRange() <= 82){
        shooter.fireNote(4000 , 1750);
      }
      else if(targeting.calcRange() <= 91){
        shooter.fireNote(4000 , 1600);
      }
      else{
        shooter.fireNote(4000 , 1500);
      }
    }
  }
}