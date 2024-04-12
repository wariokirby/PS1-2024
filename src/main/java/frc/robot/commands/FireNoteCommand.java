// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;

public class FireNoteCommand extends Command {
  private final Shooter shooter;
  private final Targeting targeting;
  
  /** Creates a new FireNote. */
  public FireNoteCommand(Shooter shooter , Targeting targeting) {
    this.shooter = shooter;
    this.targeting = targeting;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(targeting.getValidTarget() == 0){
      shooter.fireNote(2000 , 2000);
    }
    else{
      newShooter();
    }
 }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void newShooter(){
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

  public void oldShooter(){
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
