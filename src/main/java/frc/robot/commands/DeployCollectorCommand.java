// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class DeployCollectorCommand extends Command {
  private Collector collector;
  /** Creates a new DeployCollector. */
  public DeployCollectorCommand(Collector collector) {
    this.collector = collector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collector.manual(0, -.66);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return collector.isDown();
  }
}
