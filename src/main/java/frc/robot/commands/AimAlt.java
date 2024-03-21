// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Targeting;

public class AimAlt extends Command {
  private SwerveDrive drivetrain;
  private Targeting targeting;
  private boolean isAuto;

  /** Creates a new Aim. */
  public AimAlt(SwerveDrive drivetrain, Targeting targeting, boolean isAuto) {
    this.drivetrain = drivetrain;
    this.targeting = targeting;
    this.isAuto = isAuto;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.disableFieldOriented();
    targeting.changeTag(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {//dist 20.25
    double angle = Math.atan(20.25 / targeting.calcRange());
    angle = Math.toDegrees(angle);
    if(targeting.calcRange() < 90) {
      drivetrain.podDriver((angle - targeting.getX())/50, 0, (0 - drivetrain.getYaw()) / 50.0);
    }
    else {
      drivetrain.podDriver((angle - targeting.getX())/50, -.5, (0 - drivetrain.getYaw()) / 50.0);
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
    if(isAuto){
      return (targeting.calcRange() < 71 && Math.abs(targeting.getX()) < 2) || targeting.getValidTarget() == 0;
    }
    else{
      return false;
    }
    
  }
}
