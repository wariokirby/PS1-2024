// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive drive = new SwerveDrive();
  private final Shooter shooter = new Shooter();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox =
      new CommandXboxController(0);
  private final CommandXboxController xboxOperator =
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(Commands.run(
      () -> drive.podDriver(-xbox.getLeftX(), -xbox.getLeftY() , xbox.getRightX()),
      drive
      ));

    shooter.setDefaultCommand(Commands.run(
      () -> shooter.fireNote(-xboxOperator.getLeftY() , -xboxOperator.getRightY()),
      shooter
      ));

    // Configure the trigger bindings
    configureBindings();
  }



  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    xbox.a().onTrue(Commands.runOnce(drive :: resetYaw , drive));

    xboxOperator.a().onTrue(Commands.run(
      () -> shooter.fireNote(.5 , -xboxOperator.getRightY()),
      shooter
      ));
    
    xboxOperator.b().onTrue(Commands.runOnce(shooter :: stopShooter, shooter));

    /*xbox.x().onTrue(Commands.runOnce(
      () -> drive.switcher(2),
      drive
      ));
      
    xbox.y().onTrue(Commands.runOnce(
      () -> drive.switcher(3),
      drive
      ));
        
    xbox.b().onTrue(Commands.runOnce(
      () -> drive.switcher(4),
      drive
      ));*/
    
    /*xbox.start().onTrue(Commands.run(
      () -> drive.podTester(-xbox.getLeftY(), (xbox.getRightX() / 2)), 
      drive
      ));

    xbox.back().onTrue(Commands.runOnce(drive::stop, drive));*/

    /*xbox.povUp().onTrue(Commands.run(
      () -> drive.setDirection(0),
      drive
      ));

    xbox.povRight().onTrue(Commands.run(
      () -> drive.setDirection(-45),
      drive
      ));

    xbox.povLeft().onTrue(Commands.run(
      () -> drive.setDirection(45),
      drive
      ));

    xbox.povDown().onTrue(Commands.run(
      () -> drive.setDirection(135),
      drive
      ));*/
    
          // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  //  new Trigger(m_exampleSubsystem::exampleCondition)
  //      .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
