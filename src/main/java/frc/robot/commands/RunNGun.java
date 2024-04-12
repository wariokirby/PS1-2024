// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Targeting;

public class RunNGun extends Command {
  private SwerveDrive drivetrain;
  private Targeting targeting;
  private CommandXboxController xbox;
  private boolean usingAlt;

  /** Creates a new Aim. */
  public RunNGun(SwerveDrive drivetrain, Targeting targeting , CommandXboxController xbox) {
    this.drivetrain = drivetrain;
    this.targeting = targeting;
    this.xbox = xbox;
    usingAlt = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
public void execute() {
    if(targeting.getValidTarget() == 0){
      if(usingAlt){
        targeting.changeTag(0);
        usingAlt = false;
      }
      else{
        targeting.changeTag(1);
        usingAlt = true;
      }
      drivetrain.podDriver(-xbox.getLeftX(), -xbox.getLeftY(), -xbox.getRightX());
    }
    else {
      drivetrain.podDriver(-xbox.getLeftX(), -xbox.getLeftY(),  -(targeting.getX())/40.0);
    }      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.enableFieldOriented();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
