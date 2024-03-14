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
  private boolean noLimelight;
  
  /** Creates a new FireNote. */
  public FireNoteCommand(Shooter shooter , Targeting targeting , boolean noLimelight) {
    this.shooter = shooter;
    this.targeting = targeting;
    this.noLimelight = noLimelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(noLimelight || targeting.calcRange() <= 50){
      shooter.fireNote(2000 , 3000);
     }
    else if(targeting.calcRange() <= 66){
      shooter.fireNote(2000 , 2000);
    }
    else if(targeting.calcRange() <= 75){
      shooter.fireNote(4000 , 1800);
    }
    else if(targeting.calcRange() <= 90){
      shooter.fireNote(4000 , 1500);
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
}
