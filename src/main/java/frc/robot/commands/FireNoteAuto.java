// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;

public class FireNoteAuto extends Command {
  private Shooter shooter;
  private Collector collector;
  private Targeting targeting;
  private boolean noLimelight;
  private boolean secondShot;
  private int timer;
  /** Creates a new FireNote. */
  public FireNoteAuto(Shooter shooter , Collector collector , Targeting targeting, boolean noLimelight , boolean secondShot) {
    this.shooter = shooter;
    this.collector = collector;
    this.targeting = targeting;
    this.noLimelight = noLimelight;
    this.secondShot = secondShot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter , collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targeting.changeTag(0);
    timer = 75;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(noLimelight && !secondShot){
      shooter.fireNote(2000 , 4000);
      timer--;
      if(timer <= 25){
        collector.fire();
      }
    }
    else if(targeting.calcRange() < 30){
      shooter.fireNote(2000 , 4000);
      timer--;
      if(timer <= 25){
        collector.fire();
      }
    }
    else if(targeting.calcRange() < 36 || secondShot){
      shooter.fireNote(2000 , 2000);
      timer--;
      if(timer <= 25){
        collector.fire();
      }
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
    if(timer <= 0){
      return true;
    }
    else{
      return false;
    }
 }
}
