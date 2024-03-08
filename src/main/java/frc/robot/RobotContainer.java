// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimAuto;
import frc.robot.commands.Aim;
import frc.robot.commands.AutoCruise;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.Autos;
import frc.robot.commands.DeployCollectorCommand;
import frc.robot.commands.FireNoteAuto;
import frc.robot.commands.FireNoteCommand;
import frc.robot.commands.NotelSeeker;
import frc.robot.commands.RetractCollectorCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Finder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Targeting;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
  * Praj Box Documentation
 * these are the button Id's for each switch
 * safety switch 1
 * right 3way up 2
 * right 3way down 3
 * button 4
 * left 3way up 5
 * left 3way down 6
 * white switch 7
*/
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive drive = new SwerveDrive();
  private final Shooter shooter = new Shooter();
  private final Collector collector = new Collector();
  private final Climber climber = new Climber();
  private final Targeting targeting = new Targeting();
  private final Finder finder = new Finder();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox =
      new CommandXboxController(0);
  private final CommandXboxController xboxOperator =
      new CommandXboxController(1);
  private final Joystick prajBox =
      new Joystick(2);
  private Trigger enableClimber = new JoystickButton(prajBox, 1);
  private Trigger overrideCollector = new JoystickButton(prajBox, 7);


  private final Command justLeave = new AutoCruise(1, 0, 0, 3.5, drive);
  private final Command justShoot = new FireNoteAuto(shooter, collector , targeting, true, false);
  private final SequentialCommandGroup shootLeave = new SequentialCommandGroup(
    justShoot,   
    justLeave
  );
  private final SequentialCommandGroup shootLeaveOffangleRight = new SequentialCommandGroup(
    new Aim(0, drive, targeting),
    Commands.deadline(new FireNoteAuto(shooter, collector , targeting, false, false), new Aim(0, drive, targeting)), 
    new AutoCruise(1, -45, 0, 3.5, drive),
    new AutoCruise(.5, 0 , 0, 5, drive)
  );
  private final SequentialCommandGroup shootLeaveOffangleLeft = new SequentialCommandGroup(
    new Aim(0, drive, targeting),
    Commands.deadline(new FireNoteAuto(shooter, collector , targeting, false, false), new Aim(0, drive, targeting)), 
    new AutoCruise(1, 45, 0, 3.5, drive),
    new AutoCruise(.5, 0 , 0, 2, drive)
  );
  private final SequentialCommandGroup twoNoteDR = new SequentialCommandGroup(//ends at 36"
    new FireNoteAuto(shooter, collector , targeting, true, false), 
    new DeployCollectorCommand(collector),
    Commands.race(new AutoCruise(1, 0, 0, 3, drive), new AutoPickup(collector)),
    new RetractCollectorCommand(collector),
    new FireNoteAuto(shooter, collector , targeting, true, true)
  );
  private final SequentialCommandGroup twoNote = new SequentialCommandGroup(
    Commands.deadline(new FireNoteAuto(shooter, collector , targeting, false, false), new Aim(0, drive, targeting)), 
    new DeployCollectorCommand(collector),
    Commands.race(new AutoPickup(collector), new AutoCruise(1, 0, 0, (30 / 12.0), drive)),
    new NotelSeeker(drive, finder, collector),
    new RetractCollectorCommand(collector),
    new Aim(0, drive, targeting),
    Commands.deadline(new FireNoteAuto(shooter, collector , targeting, false , false), new Aim(0, drive, targeting))
  );
  private final SequentialCommandGroup threeNote = new SequentialCommandGroup(//24.5 degrees
    twoNote,
    new DeployCollectorCommand(collector),
    Commands.race(new AutoPickup(collector), new AutoCruise(1, 24.5, 24.5, (55 / 12.0), drive)),
    new NotelSeeker(drive, finder, collector),
    new RetractCollectorCommand(collector),
    new Aim(0, drive, targeting),
    Commands.deadline(new FireNoteAuto(shooter, collector , targeting, false , false), new Aim(0, drive, targeting))
  );
 
  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.setDefaultOption("Shoot and Leave", shootLeave);
    autoChooser.addOption("Right Offangle shoot", shootLeaveOffangleRight);
    autoChooser.addOption("Left Offangle shoot", shootLeaveOffangleLeft);
    autoChooser.addOption("Just Shoot", justShoot);
    autoChooser.addOption("Just Leave", justLeave);
    autoChooser.addOption("Two Note Maybe?", twoNoteDR);
    autoChooser.addOption("Two Note", twoNote);

    SmartDashboard.putData("Auto Choose" , autoChooser);

    drive.setDefaultCommand(Commands.run(
      () -> drive.podDriver(-xbox.getLeftX(), -xbox.getLeftY() , xbox.getRightX()),
      drive
      ));

    collector.setDefaultCommand(Commands.run(
        () -> collector.manual(-xboxOperator.getRightY() , -xboxOperator.getLeftY()),
        collector
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
    xbox.back().onTrue(Commands.runOnce(drive :: resetYaw , drive));
    xbox.rightTrigger()
      .onTrue(Commands.runOnce(drive :: turboOn, drive))
      .onFalse(Commands.runOnce(drive :: turboOff, drive));
    xbox.leftTrigger().whileTrue(new Aim(0, drive, targeting));

//shooter
    xboxOperator.start().onTrue(Commands.run(shooter :: fireNote , shooter));
    xboxOperator.rightBumper().onTrue(new FireNoteCommand(shooter, targeting, false));
    xboxOperator.leftBumper().onTrue(Commands.runOnce(shooter :: stopShooter, shooter));
    xboxOperator.back().onTrue(Commands.run(shooter :: fireNoteAmp, shooter));
    
    xboxOperator.rightTrigger().whileTrue(Commands.run(collector :: fire , collector));
    xboxOperator.leftTrigger().whileTrue(Commands.run(collector :: intake, collector));   
    xboxOperator.a().onTrue(new FireNoteAuto(shooter, collector, targeting, false, false));

    overrideCollector
      .onTrue(Commands.runOnce(collector :: enableOverride, collector))
      .onFalse(Commands.runOnce(collector :: disableOverride, collector));

    //prajbox safety switch on activates climbers on sticks, disables collector
    //hold button to reverse
    enableClimber.whileTrue(Commands.deadline(
      Commands.run(() -> climber.runClimbers(-xboxOperator.getLeftY(), -xboxOperator.getRightY(), 
        true, prajBox),
        climber),
      Commands.run(collector :: off, collector)
      ));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    //return new SequentialCommandGroup(
    //new FireNoteAuto(shooter, collector , targeting, true, false),   
    //new AutoCruise(1, 0, 0, 4, drive)
  //);
  //return new FireNoteAuto(shooter, collector , targeting, true, false);
  return autoChooser.getSelected();
  }
}
