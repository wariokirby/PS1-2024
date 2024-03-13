// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Finder;
// import static frc.robot.Constants.*;
import frc.robot.subsystems.SwerveDrive;

public class NotelSeeker extends Command {
  /** Creates a new BallSeeker. */
  private SwerveDrive drivetrain;
  private Finder finder;
  private Collector collector;
  private int[] dize;

  public NotelSeeker(SwerveDrive drivetrain, Finder finder, Collector collector) {
    this.drivetrain = drivetrain;
    this.finder = finder;
    this.collector = collector;

    dize = new int[2];

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain , finder , collector);
  } //end BallSeeker

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.disableFieldOriented();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dize = finder.findClosestTarget();
    if (dize[0] == 160) {
      drivetrain.podDriver(0, 0, 0.5);
    } //end if 
    else {
      collector.intake();
      drivetrain.podDriver(-(dize[0] - 74) / 100.0, (dize[1] + 24) / 50, 0);
    } //end else
  } //end execute

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      new AutoCruise(.5, drivetrain.getYaw(), 0, .5, drivetrain);
    }
    collector.off();
    drivetrain.podDriver(0, 0, 0);
    drivetrain.enableFieldOriented();
  } //ends end

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return collector.getNoteDetect();
    return Math.abs(dize[0] - 74) < 5 && Math.abs(dize[1] + 24) < 5;
  } //end isFinished
}