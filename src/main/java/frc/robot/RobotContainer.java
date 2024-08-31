// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Aim;
import frc.robot.commands.AutoCruise;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.DeployClimbers;
import frc.robot.commands.DeployCollectorCommand;
import frc.robot.commands.FireNoteAuto;
import frc.robot.commands.FireNoteAutoFaster;
import frc.robot.commands.FireNoteCommand;
//import frc.robot.commands.NoteGrabber;
//import frc.robot.commands.NotelSeeker;
import frc.robot.commands.RetractCollectorCommand;
import frc.robot.commands.RunNGun;
import frc.robot.commands.ShooterAlwaysOn;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
//import frc.robot.subsystems.Finder;
//import frc.robot.subsystems.FinderL;
import frc.robot.subsystems.Pathfinder;
import frc.robot.subsystems.PhotonFinder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Targeting;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final Climber climber = new Climber();
  private final Targeting targeting = new Targeting();
  private final PhotonFinder finder = new PhotonFinder();
  private final Pathfinder pathfinder = new Pathfinder(drive, targeting);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox =
      new CommandXboxController(0);
  private final CommandXboxController xboxOperator =
      new CommandXboxController(1);
  private final Joystick prajBox =
      new Joystick(2);
  private Trigger enableClimber = new JoystickButton(prajBox, 1);
  private Trigger overrideCollector = new JoystickButton(prajBox, 7);
  private final Collector collector = new Collector(xbox , xboxOperator);

  private final double A_SPEED = .5;

  private final double A_PAUSE = .1;
  private final Command justLeave = new AutoCruise(1, 0, 0, 3, drive);

  private final SequentialCommandGroup justShoot = new SequentialCommandGroup(
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNote , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),
    Commands.runOnce(shooter :: stopShooter, shooter),
    Commands.runOnce(collector :: off, collector)
  );

  //counterclockwise is supposedly positive
  //first shot, 44.2 left for red, right for blue 
  //second shot 31.57  ^^^ 
  private final SequentialCommandGroup twoNoteRed = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    new AutoCruise(0, 0, 44.2, 0, drive),
    new FireNoteAuto(shooter, collector , drive , targeting, false , false),
    Commands.parallel(new AutoCruise(0, 0, 0, 0, drive), new DeployCollectorCommand(collector)),
    Commands.deadline(new AutoCruise(.25, 0, 0, (41 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(.1),//end drive segment
    Commands.parallel(new RetractCollectorCommand(collector), new AutoCruise(A_SPEED, 180, 44.2, (40 / 12.0), drive)),
    new FireNoteAuto(shooter, collector , drive , targeting, true , false)
  );
  private final SequentialCommandGroup twoNoteBlue = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    new AutoCruise(0, 0, -44.2, 0, drive),
    new FireNoteAuto(shooter, collector , drive , targeting, false , false),
    Commands.parallel(new AutoCruise(0, 0, 0, 0, drive), new DeployCollectorCommand(collector)),
    Commands.deadline(new AutoCruise(.25, 0, 0, (41 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(.1),//end drive segment
    Commands.parallel(new RetractCollectorCommand(collector), new AutoCruise(A_SPEED, 180, -44.2, (40 / 12.0), drive)),
    new FireNoteAuto(shooter, collector , drive , targeting, false , false)
  );

  private final SequentialCommandGroup threeNoteRed = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    new AutoCruise(0, 0, 44.2, 0, drive),
    new FireNoteAuto(shooter, collector , drive , targeting, false , false),
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (39 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new RetractCollectorCommand(collector), new AutoCruise(0, 0, 44.2, 0, drive)),
    new FireNoteAuto(shooter, collector , drive , targeting, true , false),
    Commands.parallel(new AutoCruise(A_SPEED, 90, 21.76, (32 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 15, 15, (284 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, -159, 44.2, (269.72 / 12.0), drive) , new RetractCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    new AutoCruise(A_SPEED, -90, 44.2, (32 / 12.0), drive),
    new FireNoteAuto(shooter, collector , drive , targeting, false , false)
  );
  private final SequentialCommandGroup threeNoteBlue = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    new AutoCruise(0, 0, -44.2, 0, drive),
    new FireNoteAuto(shooter, collector , drive , targeting, false , false),
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (39 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(.1),//end drive segment
    Commands.parallel(new RetractCollectorCommand(collector), new AutoCruise(0, 0, -44.2, 0, drive)),
    new FireNoteAuto(shooter, collector , drive , targeting, true , false),
    Commands.parallel(new AutoCruise(A_SPEED, -90, -21.76, (32 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, -15, -15, (284 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 159, -44.2, (269.72 / 12.0), drive) , new RetractCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    new AutoCruise(A_SPEED, 90, -44.2, (32 / 12.0), drive),
    new FireNoteAuto(shooter, collector , drive , targeting, false , false)
  );

  private final SequentialCommandGroup denyCenterRed = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    new AutoCruise(0, 0, 44.2, 0, drive),
    new FireNoteAuto(shooter, collector , drive , targeting, false , false),
    Commands.parallel(new AutoCruise(A_SPEED, 90, 21.76, (32 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(.1),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 15, 15, (284 / 12.0), drive) , new AutoPickup(collector)),
    //Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 180, 39.27, (74.16 / 12.0), drive) , new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector, drive, targeting, true, false),
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, -32.67, -32.67, (112.28 / 12.0), drive) , new AutoPickup(collector)),
    //Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 164.91, 21.76, (82.72 / 12.0), drive) , new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector, drive, targeting, true, false),
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, -47.23, -47.23, (125.56 / 12.0), drive) , new AutoPickup(collector)),
    //Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 180, 0, (74 / 12.0), drive) , new RetractCollectorCommand(collector))
  );
  private final SequentialCommandGroup denyCenterBlue = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    new AutoCruise(0, 0, -44.2, 0, drive),
    new FireNoteAuto(shooter, collector , drive , targeting, false , false),
    Commands.parallel(new AutoCruise(A_SPEED, -90, -21.76, (32 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(.1),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, -15, -15, (284 / 12.0), drive) , new AutoPickup(collector)),
    //Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 180, -39.27, (74.16 / 12.0), drive) , new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector, drive, targeting, true, false),
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 35.67, 35.67, (112.28 / 12.0), drive) , new AutoPickup(collector)),
   // Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, -164.91, -21.76, (82.72 / 12.0), drive) , new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector, drive, targeting, true, false),
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 47.23, 47.23, (125.56 / 12.0), drive) , new AutoPickup(collector)),
    //Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 180, 0, (74 / 12.0), drive) , new RetractCollectorCommand(collector))
  );

  //get center to 2nd note position 62.33
  //for 3rd note from 2nd note: 210.6 back, 75 side, 223.6 hyp at 19.6 degrees then remove 20.5
  private final SequentialCommandGroup threeNoteCenterRed = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNote , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),
    Commands.runOnce(shooter :: stopShooter, shooter),
    Commands.runOnce(collector :: off, collector),//end note 1
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41.08 / 12), drive) , new AutoPickup(collector)),//62.33
    new RetractCollectorCommand(collector),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNoteWall , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),
    Commands.runOnce(shooter :: stopShooter, shooter),
    Commands.runOnce(collector :: off, collector),//end note 2
    Commands.parallel(new AutoCruise(A_SPEED, 0, 0, (21.25 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 19.6, 19.6, (223.6 / 12), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 162.23, -17.77, (259.27 / 12.0), drive), new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector , drive, targeting, false , false)
  );
  private final SequentialCommandGroup threeNoteCenterBlue = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNote , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),
    Commands.runOnce(shooter :: stopShooter, shooter),
    Commands.runOnce(collector :: off, collector),//end note 1
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41.08 / 12), drive) , new AutoPickup(collector)),//62.33
    new RetractCollectorCommand(collector),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNoteWall , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),
    Commands.runOnce(shooter :: stopShooter, shooter),
    Commands.runOnce(collector :: off, collector),//end note 2
    Commands.parallel(new AutoCruise(A_SPEED, 0, 0, (21.25 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, -19.6, -19.6, (223.6 / 12), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, -162.23, 17.77, (259.27 / 12.0), drive), new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector , drive, targeting, false , false)
  );

  private final SequentialCommandGroup fourNoteRed = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNote , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),
    Commands.runOnce(shooter :: stopShooter, shooter),
    Commands.runOnce(collector :: off, collector),//end note 1
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41.08 / 12), drive) , new AutoPickup(collector)),//62.33
    new RetractCollectorCommand(collector),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNoteWall , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),
    Commands.runOnce(shooter :: stopShooter, shooter),
    Commands.runOnce(collector :: off, collector),//end note 2
    Commands.parallel(new AutoCruise(A_SPEED, 0, 0, (21.25 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 19.6, 19.6, (223.6 / 12), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 162.23, -17.77, (259.27 / 12.0), drive), new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector , drive, targeting, false , false),
     new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, -42.68, -42.68, (46.26 / 12.0), drive) , new AutoPickup(collector)),
    Commands.parallel(new AutoCruise(A_SPEED, 137.32, -17.77, (46.26 / 12.0), drive), new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector, drive, targeting, false , false)
  );
  private final SequentialCommandGroup fourNoteBlue = new SequentialCommandGroup(
    Commands.runOnce(drive::resetYaw, drive),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNote , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),
    Commands.runOnce(shooter :: stopShooter, shooter),
    Commands.runOnce(collector :: off, collector),//end note 1
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41.08 / 12), drive) , new AutoPickup(collector)),//62.33
    new RetractCollectorCommand(collector),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNoteWall , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),
    Commands.runOnce(shooter :: stopShooter, shooter),
    Commands.runOnce(collector :: off, collector),//end note 2
    Commands.parallel(new AutoCruise(A_SPEED, 0, 0, (21.25 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, -19.6, -19.6, (223.6 / 12), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, -162.23, 17.77, (259.27 / 12.0), drive), new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector , drive, targeting, false , false),
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 42.68, 42.68, (46.26 / 12.0), drive) , new AutoPickup(collector)),
    Commands.parallel(new AutoCruise(A_SPEED, -137.32, 17.77, (46.26 / 12.0), drive), new RetractCollectorCommand(collector)),
    new FireNoteAuto(shooter, collector, drive, targeting, false , false)
  );


  SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.setDefaultOption("Deny Center Red", denyCenterRed);
    autoChooser.addOption("Deny Center Blue", denyCenterBlue);
    autoChooser.addOption("Three Note Red", threeNoteRed);
    autoChooser.addOption("Three Note Blue", threeNoteBlue);
    autoChooser.addOption("Two Note Red", twoNoteRed);
    autoChooser.addOption("Two Note Blue", twoNoteBlue);
    autoChooser.addOption("Just Shoot", justShoot);
    autoChooser.addOption("Just Leave", justLeave);
    autoChooser.addOption("3 Note Center Red", threeNoteCenterRed);
    autoChooser.addOption("3 Note Center Blue", threeNoteCenterBlue);
    autoChooser.addOption("Be Wario Red", fourNoteRed);
    autoChooser.addOption("Be Wario Blue", fourNoteBlue);

    SmartDashboard.putData("Auto Choose" , autoChooser);

    drive.setDefaultCommand(Commands.run(
      () -> drive.podDriver(-xbox.getLeftX(), -xbox.getLeftY() , -xbox.getRightX() , true , true),
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
    xbox.a().onTrue(Commands.runOnce(drive :: resetYaw , drive));
    xbox.rightTrigger()
      .onTrue(Commands.runOnce(drive :: turboOn, drive))
      .onFalse(Commands.runOnce(drive :: turboOff, drive));
    xbox.leftBumper().whileTrue(new Aim(0, drive, targeting, false));
    xbox.leftTrigger().whileTrue(new RunNGun(drive, targeting, xbox));
    //xbox.a().onTrue(new SequentialCommandGroup(new NotelSeeker(drive, finder, collector) , new NoteGrabber(drive, collector)));
    xbox.b().onTrue(Commands.runOnce(drive :: stop, drive));

//shooter
    xboxOperator.x().onTrue(Commands.run(shooter :: fireNote , shooter));
    xboxOperator.rightBumper().onTrue(new FireNoteCommand(shooter, targeting));
    xboxOperator.leftBumper().onTrue(Commands.runOnce(shooter :: stopShooter, shooter));
    xboxOperator.back().onTrue(Commands.run(shooter :: fireNoteAmp, shooter));
    xboxOperator.y().onTrue(Commands.run(shooter :: fireNoteWall, shooter));
    xboxOperator.b().onTrue(Commands.run(shooter :: fireNoteAmp, shooter));

    xboxOperator.rightTrigger().whileTrue(Commands.run(collector :: fire , collector));
    xboxOperator.leftTrigger().whileTrue(Commands.run(collector :: intake, collector));   
    xboxOperator.a().onTrue(new FireNoteAuto(shooter, collector, drive , targeting, false , false));

    overrideCollector
      .onTrue(Commands.runOnce(collector :: enableOverride, collector))
      .onFalse(Commands.runOnce(collector :: disableOverride, collector));

    //prajbox safety switch on activates climbers on sticks, disables collector
    //hold button to reverse
    /*enableClimber.whileTrue(Commands.deadline(
      Commands.run(() -> climber.runClimbers(-xboxOperator.getLeftY(), -xboxOperator.getRightY(), 
        true, prajBox),
        climber),
      Commands.run(collector :: off, collector)
      ));*/

    enableClimber.whileTrue(new SequentialCommandGroup(
      new DeployClimbers(climber) ,
      Commands.deadline(
      Commands.run(() -> climber.runClimbers(-xboxOperator.getLeftY(), -xboxOperator.getRightY(), 
        true, prajBox),
        climber),
      Commands.run(collector :: off, collector)
      ))
    );

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



    private final SequentialCommandGroup twoNoteRedA = new SequentialCommandGroup(
    Commands.deadline(new AutoCruise(0, 0, 44.2, 0, drive) , new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//1
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 180, 44.2, (40 / 12.0), drive) , new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false)//2
  );
  private final SequentialCommandGroup twoNoteBlueA = new SequentialCommandGroup(
    Commands.deadline(new AutoCruise(0, 0, -44.2, 0, drive), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//1
    Commands.parallel(new DeployCollectorCommand(collector), new AutoCruise(0, 0, 0, 0, drive)),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 180, -44.2, (40 / 12.0), drive), new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
   new FireNoteAutoFaster(shooter, collector , drive , targeting, false)//2
  );

  private final SequentialCommandGroup threeNoteRedA = new SequentialCommandGroup(
    Commands.deadline(new AutoCruise(0, 0, 44.2, 0, drive), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//1
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (39 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(0, 0, 44.2, 0, drive), new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//2
    Commands.parallel(new AutoCruise(A_SPEED, 90, 21.76, (32 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 21, 21, (284 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, -159, 44.2, (269.72 / 12.0), drive) , new RetractCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, -90, 44.2, (32 / 12.0), drive), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false)//3
  );
  private final SequentialCommandGroup threeNoteBlueA = new SequentialCommandGroup(
    Commands.deadline(new AutoCruise(0, 0, -44.2, 0, drive), new ShooterAlwaysOn(shooter, targeting)),
   new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//1
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (39 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(0, 0, -44.2, 0, drive), new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//2
    Commands.parallel(new AutoCruise(A_SPEED, -90, -21.76, (32 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, -21, -21, (284 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 159, -44.2, (269.72 / 12.0), drive) , new RetractCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 90, -44.2, (32 / 12.0), drive), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false)//3
  );

  private final SequentialCommandGroup denyCenterRedA = new SequentialCommandGroup(
    Commands.deadline(new AutoCruise(0, 0, 44.2, 0, drive), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//1
    Commands.parallel(new AutoCruise(A_SPEED, 90, 21.76, (32 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 21, 21, (284 / 12.0), drive) , new AutoPickup(collector), new ShooterAlwaysOn(shooter, targeting)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 180, 39.27, (74.16 / 12.0), drive) , new RetractCollectorCommand(collector)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//2
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, -32.67, -32.67, (112.28 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 164.91, 21.76, (82.72 / 12.0), drive) , new RetractCollectorCommand(collector)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//3
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, -47.23, -47.23, (125.56 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 180, 0, (74 / 12.0), drive) , new RetractCollectorCommand(collector))//4 not shot
  );
  private final SequentialCommandGroup denyCenterBlueA = new SequentialCommandGroup(
    Commands.deadline(new AutoCruise(0, 0, -44.2, 0, drive), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//1
    Commands.parallel(new AutoCruise(A_SPEED, -90, -21.76, (32 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, -21, -21, (284 / 12.0), drive) , new AutoPickup(collector), new ShooterAlwaysOn(shooter, targeting)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 180, -39.27, (74.16 / 12.0), drive) , new RetractCollectorCommand(collector)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//2
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 32.67, 32.67, (112.28 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, -164.91, -21.76, (82.72 / 12.0), drive) , new RetractCollectorCommand(collector)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//3
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 47.23, 47.23, (125.56 / 12.0), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 180, 0, (74 / 12.0), drive) , new RetractCollectorCommand(collector))//4 not shot
  );

  //get center to 2nd note position 62.33
  //for 3rd note from 2nd note: 210.6 back, 75 side, 223.6 hyp at 19.6 degrees then remove 20.5
  private final SequentialCommandGroup threeNoteCenterRedA = new SequentialCommandGroup(
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNote , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),//1
    Commands.runOnce(collector :: off, collector),
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41.08 / 12), drive) , new AutoPickup(collector), new ShooterAlwaysOn(shooter, targeting)),//62.33
    Commands.deadline(new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNoteWall , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),//2
    Commands.runOnce(collector :: off, collector),
    Commands.parallel(new AutoCruise(A_SPEED, 0, 0, (21.25 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 19.6, 19.6, (223.6 / 12), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 162.23, -17.77, (259.27 / 12.0), drive), new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false)//3
  );
  private final SequentialCommandGroup threeNoteCenterBlueA = new SequentialCommandGroup(
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNote , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),//1
    Commands.runOnce(collector :: off, collector),//end note 1
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41.08 / 12), drive) , new AutoPickup(collector), new ShooterAlwaysOn(shooter, targeting)),//62.33
    new RetractCollectorCommand(collector),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNoteWall , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),//2
    Commands.runOnce(collector :: off, collector),//end note 2
    Commands.parallel(new AutoCruise(A_SPEED, 0, 0, (21.25 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, -19.6, -19.6, (223.6 / 12), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, -162.23, 17.77, (259.27 / 12.0), drive), new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false)//3
  );

  private final SequentialCommandGroup fourNoteRedA = new SequentialCommandGroup(
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNote , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),//1
    Commands.runOnce(collector :: off, collector),
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41.08 / 12), drive) , new AutoPickup(collector), new ShooterAlwaysOn(shooter, targeting)),//62.33
    Commands.deadline(new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNoteWall , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),//2
    Commands.runOnce(collector :: off, collector),
    Commands.parallel(new AutoCruise(A_SPEED, 0, 0, (21.25 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, 19.6, 19.6, (223.6 / 12), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, 162.23, -17.77, (259.27 / 12.0), drive), new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//3
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, -42.68, -42.68, (46.26 / 12.0), drive) , new AutoPickup(collector)),
    Commands.parallel(new AutoCruise(A_SPEED, 137.32, -17.77, (46.26 / 12.0), drive), new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false)//4
  );
  private final SequentialCommandGroup fourNoteBlueA = new SequentialCommandGroup(
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNote , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),//1
    Commands.runOnce(collector :: off, collector),//end note 1
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 0, 0, (41.08 / 12), drive) , new AutoPickup(collector), new ShooterAlwaysOn(shooter, targeting)),//62.33
    new RetractCollectorCommand(collector),
    Commands.deadline(new WaitCommand(.8), Commands.run(shooter :: fireNoteWall , shooter)),
    Commands.deadline(new WaitCommand(.5), Commands.run(collector :: fire , collector)),//2
    Commands.runOnce(collector :: off, collector),//end note 2
    Commands.parallel(new AutoCruise(A_SPEED, 0, 0, (21.25 / 12.0), drive), new DeployCollectorCommand(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.deadline(new AutoCruise(A_SPEED, -19.6, -19.6, (223.6 / 12), drive) , new AutoPickup(collector)),
    Commands.waitSeconds(A_PAUSE),//end drive segment
    Commands.parallel(new AutoCruise(A_SPEED, -162.23, 17.77, (259.27 / 12.0), drive), new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false),//3
    new DeployCollectorCommand(collector),
    Commands.deadline(new AutoCruise(A_SPEED, 42.68, 42.68, (46.26 / 12.0), drive) , new AutoPickup(collector)),
    Commands.parallel(new AutoCruise(A_SPEED, -137.32, 17.77, (46.26 / 12.0), drive), new RetractCollectorCommand(collector), new ShooterAlwaysOn(shooter, targeting)),
    new FireNoteAutoFaster(shooter, collector , drive , targeting, false)//4
  );

}
