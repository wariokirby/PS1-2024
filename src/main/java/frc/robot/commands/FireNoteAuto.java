// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FireNoteAuto extends Command {
  private Shooter shooter;
  private Collector collector;
  private Targeting targeting;
  private boolean noLimelight;
  private boolean secondShot;
  private int timer;
  private boolean delay;
  /** Creates a new FireNote. */
  public FireNoteAuto(Shooter shooter , Collector collector , Targeting targeting, boolean noLimelight , boolean secondShot , boolean delay) {
    this.shooter = shooter;
    this.collector = collector;
    this.targeting = targeting;
    this.noLimelight = noLimelight;
    this.secondShot = secondShot;
    this.delay  = delay;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter , collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targeting.changeTag(0);
    if(delay){
      timer = 150;
    }
    else{
      timer = 65;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(noLimelight && !secondShot){
      shooter.fireNote(2000 , 4000);
    }
    else if((noLimelight && secondShot) || targeting.getValidTarget() == 0){
      shooter.fireNote(2000 , 3000);
    }
    else if(targeting.calcRange() <= 45){
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

   timer--;
    if(timer <= 25){
      collector.fire();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    collector.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("autoTimer" , timer);
    return timer <= 0;
 }
}
