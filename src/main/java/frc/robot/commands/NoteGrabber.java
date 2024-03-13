// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.SwerveDrive;

public class NoteGrabber extends Command {
  private SwerveDrive drivetrain;
  private Collector collector;

  /** Creates a new NoteGrabber. */
  public NoteGrabber(SwerveDrive drivetrain, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain , collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetDistances(false);
    drivetrain.disableFieldOriented();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collector.intake();
    drivetrain.podDriver(0, .5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.off();
    drivetrain.podDriver(0, 0, 0);
    drivetrain.enableFieldOriented();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getAverageDistance() > 6;
  }
}
