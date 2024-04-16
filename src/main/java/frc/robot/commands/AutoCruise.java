// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class AutoCruise extends Command {
  private final SwerveDrive drivetrain;
  private double speed;
  private double heading;
  private double turn;
  private double distance;
  private double x1;
  private double y1;
  private double xSlow;
  private double ySlow;
  private double xMid;
  private double yMid;

  /** Creates a new AutoCruise. */
  public AutoCruise(double speed , double direction , double turn , double distance , SwerveDrive drivetrain) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.heading = direction * Math.PI / 180;
    this.turn = turn;
    this.distance = distance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.enableFieldOriented();
    drivetrain.turboOn();
    //drivetrain.resetYaw();
    y1 = speed * Math.cos(heading);
    x1 = speed * Math.sin(heading);    
    ySlow = .25 * Math.cos(heading);
    xSlow = .25 * Math.sin(heading);
    yMid = .5 * Math.cos(heading);
    xMid = .5 * Math.sin(heading);
    drivetrain.resetDistances(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drivetrain.getAverageDistance() < 2 || distance - drivetrain.getAverageDistance() < 2){
      drivetrain.podDriver(xSlow, ySlow, ((turn - drivetrain.getYaw()) / 100.0) , false , false);
    }
    else if(speed > .5 && (drivetrain.getAverageDistance() < 3 || distance - drivetrain.getAverageDistance() < 3)){
      drivetrain.podDriver(xMid, yMid, ((turn - drivetrain.getYaw()) / 100.0) , false , false);
    }
    else{
      drivetrain.podDriver(x1, y1, ((turn - drivetrain.getYaw()) / 100.0) , false , false);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getAverageDistance() >= distance && Math.abs(turn - drivetrain.getYaw()) < 2;
  }
}
