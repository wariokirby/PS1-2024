// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PhotonFinder;
//import frc.robot.subsystems.FinderL;
// import static frc.robot.Constants.*;
import frc.robot.subsystems.SwerveDrive;

public class NotelSeeker extends Command {
  /** Creates a new BallSeeker. */
  private SwerveDrive drivetrain;
  //private FinderL finder;
  private PhotonFinder finder;
  private Collector collector;
  private double[] dize;

  public NotelSeeker(SwerveDrive drivetrain, PhotonFinder finder, Collector collector) {
    this.drivetrain = drivetrain;
    this.finder = finder;
    this.collector = collector;

    dize = new double[3];

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
    if(dize[1] < 0){
      dize[1] = 0;
    }
    if (dize[0] == 160) {
      drivetrain.podDriver(0, 0, 0.5 , false , false);
    } //end if 
    else {
      collector.intake();
      drivetrain.podDriver(-(dize[0] - 74) / 100.0, (dize[1] + 24) / 100.0, 0 , false , false);
    } //end else
  } //end execute

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.off();
    drivetrain.stop();
    drivetrain.enableFieldOriented();
  } //ends end

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return collector.getNoteDetect();
    return (Math.abs(dize[0] - 74) < 15 && (dize[1] + 24) < 35) || collector.getNoteDetect(); 
  } //end isFinished
}