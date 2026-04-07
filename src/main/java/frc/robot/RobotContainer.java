package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.NewtonMeter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Setpoints.Hood;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.IntakeAgitateCommand;
import frc.robot.commands.IntakeAutoCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.commands.ShootKickIndexCommand;
import frc.robot.commands.TrenchCommand;
import frc.robot.commands.UnstuckCommand;
import frc.robot.commands.slowMode;
import frc.robot.commands.unused.ShootOnTheMoveCommand;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.field.FieldConstants;
import swervelib.SwerveInputStream;
import java.util.function.Predicate;


public class RobotContainer
{

  private final SendableChooser<Command> autoChooser;
  

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  // private final TurretVisualizer turretVisualizer =
  //     new TurretVisualizer(() -> new Pose3d(drivebase.getPose()), drivebase::getFieldVelocity);

  private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
  private final IntakeArmSubsystem    intakeArm    = new IntakeArmSubsystem();

  //private final HoodSubsystem     hood           = new HoodSubsystem();
  private final FlywheelSubsystem turretFlywheel = new FlywheelSubsystem();
  private final IndexerSubsystem  indexer  = new IndexerSubsystem();
  private final AgitatorSubsystem agitator = new AgitatorSubsystem();
  private final KickerSubsystem   kicker   = new KickerSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();

  public static Timer                 timerThing           = new Timer();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController   =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
                           drivebase.getSwerveDrive(),
                           () -> m_driverController.getLeftY() * -1,
                           () -> m_driverController.getLeftX() * -1)
                       .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
                       .deadband(OperatorConstants.DEADBAND)
                       .scaleTranslation(.8)
                       .scaleRotation(0.8)
                       .allianceRelativeControl(true);


  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command autoSwerveInputStream = drivebase.driveFieldOriented(driveAngularVelocity.copy()
                                                                                   // Fallback
                                                                                   .aim(FieldConstants.Hub.getHubPose())
                                                                                   .aimHeadingOffset(Rotation2d.k180deg)
                                                                                   .aimWhile(() -> true));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("ShootCommand",
                                  new ShootKickIndexCommand(turretFlywheel,
                                                            kicker,
                                                            indexer,
                                                            agitator,
                                                            hood,
                                                            drivebase
                                                            //Setpoints.Hood.hubDegree
                                  ).withTimeout(Seconds.of(6)));
    NamedCommands.registerCommand("AimAtHub", new AutoAimCommand(drivebase, driveAngularVelocity));
    NamedCommands.registerCommand("PreShotAgitate", agitator.setDutyCycleCommand(-0.1).withTimeout(1));
    NamedCommands.registerCommand("ArmUp", intakeArm.setAngleCommand(Setpoints.Trench.intakeArmUpAngle).withTimeout(0.9));
    NamedCommands.registerCommand("ArmDown", intakeArm.setAngleCommand(Setpoints.Trench.intakeArmDownAngle).withTimeout(1.4));
    NamedCommands.registerCommand("AgitatorRun", agitator.setDutyCycleCommand(0.3).withTimeout(10));
    
    
    // Configure the trigger bindings

    configureBindings();
    driverControls();
    operatorControls();
    defaultCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    

    // Named commands do NOT run with path's. They are inbetween paths.
    

    new EventTrigger("IntakeStart").onTrue(
                                                  new IntakeAutoCommand(intakeRoller).alongWith(agitator.setDutyCycleCommand(0.3)));//.alongWith(new IntakeAutoCommand(intakeRoller))
    
    
    
    new EventTrigger("IntakeStop").onTrue(intakeRoller.stopCommand().alongWith(agitator.setDutyCycleCommand(0)));


 
    // new EventTrigger("Shoot").onTrue(

    // new EventTrigger("Shoot").onTrue(
    //                               new ShootKickIndexCommand(turretFlywheel,
    //                                                         kicker,
    //                                                         indexer,
    //                                                         agitator,
    //                                                         hood,
    //                                                         drivebase
    //                                                         //Setpoints.Hood.hubDegree
    //                               ).withTimeout(Seconds.of(10)));

                                  
    
  }

/*------------------------------- DEFAULT COMMANDS ------------------------------------------------------------------------------------------------------------------------------------ */
  public void defaultCommands()
  {

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    kicker.setDefaultCommand(kicker.setDutyCycleCommand(-0)); // Set -0.3 before on field
    agitator.setDefaultCommand(agitator.setDutyCycleCommand(0));
    indexer.setDefaultCommand(indexer.setDutyCycleCommand(-0)); // Set -0.3 before on field
    turretFlywheel.setDefaultCommand(turretFlywheel.setDutyCycle(0));
    intakeArm.setDefaultCommand(intakeArm.setDutyCycleCommand(m_operatorController::getLeftY, m_operatorController::getRightY)); //NEED TO CHANGE
    hood.setDefaultCommand(hood.setDegreeCommand(Setpoints.Intake.hoodDownAngle.in(Degrees)));

    // intakeArm.setDefaultCommand(intakeArm.setAngleCommand(Setpoints.Intake.intakeArmAngleUp));

    // Change the auto-aim to aim at our alliances hub.
    RobotModeTriggers.teleop()
                     .onTrue(Commands.runOnce(() -> driveAngularVelocity.aim(FieldConstants.Hub.getHubPose())));

        //m_operatorController.button(1).whileTrue(((new IntakeAgitateCommand(intakeArm).andThen(Commands.waitTime(Seconds.of(0.3)))).withTimeout(1.2)).repeatedly());
  }


/*-------------------------------- DRIVER CONTROLLERS ----------------------------------------------------------------------------------------------------------------------------------------------------------- */

  public void driverControls(){
    m_driverController.a().and(()->!DriverStation.isTest()).whileTrue(new AutoAimCommand(drivebase, driveAngularVelocity));
    m_driverController.rightBumper().whileTrue(new slowMode(drivebase, driveAngularVelocity));
    m_driverController.leftBumper().whileTrue(new TrenchCommand(hood));
    m_driverController.start().and(m_driverController.back()).onTrue(drivebase.zeroGyroWithAlliance());
    m_driverController.x().whileTrue(agitator.setDutyCycleCommand(-0.2));
    //reset odometry
    m_driverController.povUp().onTrue(drivebase.resetOdometryCommand(Setpoints.SwerveDrive.robotPoseAtHub));
    

  }

/*-------------------------------- OPERATOR CONTROLLERS ---------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 
  public void operatorControls(){

    m_operatorController.povRight().whileTrue(new ShootKickIndexCommand(turretFlywheel, kicker, indexer, agitator, hood, RPM.of(1500)));
    m_operatorController.x().whileTrue(new ShootKickIndexCommand(turretFlywheel, kicker, indexer, agitator, hood, RPM.of(2400)));
    m_operatorController.y().whileTrue(new ShootKickIndexCommand(turretFlywheel, kicker, indexer, agitator, hood, RPM.of(3000)));

    m_operatorController.rightTrigger(0.2).whileTrue(new ShootKickIndexCommand(turretFlywheel,
                                                                               kicker,
                                                                               indexer,
                                                                               agitator,
                                                                               hood,
                                                                               drivebase));
    //m_operatorController.rightTrigger(0.2).whileTrue(Commands.waitTime(Seconds.of(5)).andThen(((new IntakeAgitateCommand(intakeArm).andThen(Commands.waitTime(Seconds.of(0.3)))).withTimeout(1.2)).repeatedly()));

    m_operatorController.leftTrigger(0.3).whileTrue(new IntakeCommand(intakeRoller, agitator, indexer));
    m_operatorController.leftBumper().whileTrue(agitator.setDutyCycleCommand(-0.5));
    m_operatorController.rightBumper().whileTrue(((new IntakeAgitateCommand(intakeArm).andThen(Commands.waitTime(Seconds.of(0.3)))).withTimeout(1.2)).repeatedly());
    m_operatorController.povDown().onTrue(intakeArm.setAngleCommand(Setpoints.Intake.intakeArmAngleDown).withTimeout(1.3));
    m_operatorController.povUp().onTrue(intakeArm.setAngleCommand(Setpoints.Intake.intakeArmAngleUp).withTimeout(1.3));
    //m_operatorController.povUp().whileTrue(intakeArm.setAngleCommand(Setpoints.Intake.intakeArmAngleUp));
   
    m_operatorController.b().whileTrue(new OutakeCommand(intakeRoller));
    m_operatorController.a().whileTrue(new UnstuckCommand(kicker, indexer,agitator));

  }

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  //controlssss
  private void configureBindings()
  {
    var topRightOfTrench = new Pose2d().getTranslation();
    var bottomLeftOfTrench = new Pose2d().getTranslation();
    var trenchRight = new Rectangle2d(topRightOfTrench,bottomLeftOfTrench);


      var topBlueTrenchTopLeft = new Translation2d(5.238, 8.016);
      var topBlueTrenchBottomRight = new Translation2d(4.048, 6.747);
      var topBlueTrench = new Rectangle2d(topBlueTrenchTopLeft, topBlueTrenchBottomRight);

      var bottomBlueTrenchTopLeft = new Translation2d(5.146, 1.409);
      var bottomBlueTrenchBottomRight = new Translation2d(4.034, 0.045);
      var bottomBlueTrench = new Rectangle2d(bottomBlueTrenchTopLeft, bottomBlueTrenchBottomRight);

      var topRedTrenchTopLeft = new Translation2d(12.558, 8.013);
      var topRedTrenchBottomRight = new Translation2d(11.394, 6.816);
      var topRedTrench = new Rectangle2d(topRedTrenchTopLeft, topRedTrenchBottomRight);

      var bottomRedTrenchTopLeft = new Translation2d(12.539, 1.254);
      var bottomRedTrenchBottomRight = new Translation2d(11.394, 0.045);
      var bottomRedTrench = new Rectangle2d(bottomRedTrenchTopLeft, bottomRedTrenchBottomRight);

      Predicate<Pose2d> inTheTrenches = pose -> 
                                      topBlueTrench.contains(pose.getTranslation()) ||
                                      bottomBlueTrench.contains(pose.getTranslation()) ||
                                      topRedTrench.contains(pose.getTranslation()) ||
                                      bottomRedTrench.contains(pose.getTranslation())
                                      ;
                                  

      double headingDegrees = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0.0 : 180.0;
    
      SwerveInputStream stream = driveAngularVelocity.copy().withControllerHeadingAxis(
                                                              ()->  Math.cos(Degrees.of(headingDegrees).in(Radians)), 
                                                              ()->Math.sin(Degrees.of(headingDegrees).in(Radians)))
                                                            .headingWhile(true);
      
      Command driveAimedAtTrenchFieldOriented = drivebase.driveFieldOriented(stream);


    //   Trigger trench = new Trigger( 
    //                               () -> inTheTrenches.test(drivebase.getPose())
    //                               )
    //                                 .whileTrue(
    //                                             Commands.run(
    //                                                           ()-> {drivebase.setDefaultCommand(driveAimedAtTrenchFieldOriented);} 
    //                                                         )

    //                                 .finallyDo(  () -> drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity) )
    //                               );


  //  m_driverController.button(1).whileTrue(new AutoAimCommand(drivebase, driveAngularVelocity).withTimeout(5));
     //m_driverController.button(1).whileTrue((new IntakeAgitateCommand(intakeArm).andThen(Commands.waitSeconds(1))).repeatedly());
    // m_driverController.button(3).whileTrue(new ShootKickIndexCommand(turretFlywheel, kicker, indexer, agitator, Setpoints.Shooter.hubRPM).withTimeout(5));
    // m_driverController.button(4).whileTrue(new ShootKickIndexCommand(turretFlywheel, kicker, indexer, agitator, drivebase));
    // m_driverController.button(5).whileTrue(new IntakeCommand(intakeArm, intakeRoller, agitator));
    // m_driverController.button(6).whileTrue(new OutakeCommand(intakeArm, intakeRoller, agitator));
    //m_driverController.b().whileTrue(intakeArm.setAngleCommand(Degrees.of(45)));
    //m_driverController.y().whileTrue(intakeArm.setAngleCommand(Degrees.of(0)));

    // m_driverController.x().whileTrue(new ShootKickIndexCommand(turretFlywheel,
    //                                                              kicker,
    //                                                              indexer,
    //                                                              agitator,
    //                                                             // hood,
    //                                                              RPM.of(2800)
    //                                                             // Setpoints.Hood.hubDegree
    //                                                             ));
    // m_driverController.y().whileTrue(new ShootKickIndexCommand(turretFlywheel,
    //                                                              kicker,
    //                                                              indexer,
    //                                                              agitator,
    //                                                             // hood,
    //                                                              RPM.of(3350)
    //                                                             // Setpoints.Hood.hubDegree
    //                                                             ));
    // m_driverController.b().whileTrue(new ShootKickIndexCommand(turretFlywheel,
    //                                                              kicker,
    //                                                              indexer,
    //                                                              agitator,
    //                                                             // hood,
    //                                                              RPM.of(3100)
    //                                                             // Setpoints.Hood.hubDegree
    //                                                             ));
    // m_driverController.a().whileTrue(new ShootKickIndexCommand(turretFlywheel,
    //                                                              kicker,
    //                                                              indexer,
    //                                                              agitator,
    //                                                             // hood,
    //                                                              RPM.of(2950)
    //                                                             // Setpoints.Hood.hubDegree
    //                                                             ));

    // m_operatorController.x().whileTrue(kicker.setVelocityCommand(RPM.of(-1000)).alongWith(indexer.setVeloctiyCommand(RPM.of(-400))));
     // m_operatorController.x().onTrue(intakeArm.setDutyCycleCommand(-0.2).withTimeout(1));
    // m_operatorController.leftBumper().whileTrue(new IntakeCommand(intakeArm, intakeRoller, agitator));
    // m_operatorController.rightBumper().whileTrue(new OutakeCommand(intakeArm, intakeRoller, agitator));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*  */
  public Command getAutonomousCommand()
  {

    return autoChooser.getSelected();
  }

}
