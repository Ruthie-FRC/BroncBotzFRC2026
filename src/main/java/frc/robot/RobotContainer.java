package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Setpoints.Turret.Hood;
import frc.robot.Setpoints.Turret.Pivot;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Turret.HoodSubsystem;
import frc.robot.subsystems.Turret.IndexerSubsystem;
import frc.robot.subsystems.Turret.KickerSubsystem;
import frc.robot.subsystems.Turret.TurretFlywheelSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.Turret.TurretVisualizer;
// import frc.robot.systems.LoadingSystem;
// import frc.robot.systems.ScoringSystem;
import frc.robot.systems.field.FieldConstants;
import swervelib.SwerveInputStream;
import utils.FuelSim;
import yams.mechanisms.velocity.FlyWheel;
import yams.units.YUnits;


public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  // private final TurretVisualizer turretVisualizer =
  //     new TurretVisualizer(() -> new Pose3d(drivebase.getPose()), drivebase::getFieldVelocity);

  private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
  private final IntakeArmSubsystem intakeArm = new IntakeArmSubsystem();

  private final TurretSubsystem turret = new TurretSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();
  private final TurretFlywheelSubsystem turretFlywheel = new TurretFlywheelSubsystem();

  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final AgitatorSubsystem agitator = new AgitatorSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem();

  private Trigger shootAuto = new Trigger(()-> false);

  public static Timer timerThing = new Timer();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = 
        new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private static final double TRIGGER_DEADBAND = 0.1;
  private final Trigger rightTriggerDeadband = 
  new Trigger(()-> m_driverController.getRightTriggerAxis() > TRIGGER_DEADBAND);

      /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> m_driverController.getLeftY() * -1,
              () -> m_driverController.getLeftX() * -1)
          .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> -m_driverController.getLeftY(),
              () -> -m_driverController.getLeftX())
          .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));

 Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
 Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(    driveAngularVelocity.copy()
            // Fallback
            .aim(FieldConstants.Hub.getHubPose())
            .aimHeadingOffset(Rotation2d.k180deg)
            .aimWhile(m_driverController.button(1)));
  
  Command autoSwerveInputStream = drivebase.driveFieldOriented(driveAngularVelocity.copy()
            // Fallback
            .aim(FieldConstants.Hub.getHubPose())
            .aimHeadingOffset(Rotation2d.k180deg)
            .aimWhile(()-> true));

  Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngle);


 
  // private final ShootOnTheMoveCommand SOTM =
  //        new ShootOnTheMoveCommand(drivebase.getPoseSupplier(), driveAngularVelocity, FieldConstants.Hub.HubPose, turret, hood, turretFlywheel);
  /* private final LoadingSystem loading = new LoadingSystem(indexer, intakeArm, intakeRoller, drivebase, turret, agitator, kicker);
  private final ScoringSystem scoring = new ScoringSystem(indexer, intakeArm, intakeRoller, drivebase, turret, turretFlywheel, hood, kicker, agitator);*/
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    defaultCommands();
  }

  private boolean streamlinedShooting = true;

  public void defaultCommands() {

      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    //  if (RobotBase.isSimulation()) {
    //   drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    // } else {
    // }

    agitator.setDefaultCommand(agitator.setDutyCycle(0));
    indexer.setDefaultCommand(indexer.setDutyCycle(0));
    turretFlywheel.setDefaultCommand(turretFlywheel.setDutyCycle(0));
    turret.setDefaultCommand(turret.set(0));
    hood.setDefaultCommand(hood.setDutyCycle(0));
    intakeArm.setDefaultCommand(intakeArm.setAngle(intakeArm.getAngle()));
    //if not intaking, the arm is at max
    
    // climberSubsystem.setDefaultCommand(climberSubsystem.setHeight(Setpoints.Climber.startHeight));
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
    // m_driverController.button(1).whileTrue(loading.intakeBalls());
    // m_driverController.button(2).whileTrue(loading.stopIntakeBalls());
    // m_driverController.button(3).whileTrue(loading.transferBalls());
    // m_driverController.button(4).whileTrue(loading.intakeDown());
    // //m_driverController.button(5).whileTrue(scoring.score());
    // m_driverController.button(6).whileTrue(scoring.shootBall());
    
    // m_operatorController.leftTrigger().onTrue(loading.intakeBalls());//Uncomment Later
    // m_operatorController.rightTrigger().onTrue(scoring.shootBall());//UnComment Later
    m_driverController.rightTrigger().whileTrue(turretFlywheel.setDutyCycle(-0.4));
    m_driverController.a().whileTrue(kicker.setDutyCycle(-1));
    m_driverController.b().whileTrue(indexer.setDutyCycle(0.4));
    m_driverController.leftTrigger().whileTrue(agitator.setDutyCycle(0.4));
    rightTriggerDeadband.whileTrue(new ShootCommand(indexer, kicker, hood, turretFlywheel, TurretConstants.FARShooterGolRPM));
    m_driverController.button(1).whileTrue(new AutoAimCommand(drivebase, driveAngularVelocity));
    
    //m_driverController.button(5).whileTrue(intakeArm.setAngle(Setpoints.Intake.intakeArmAngleIn));
    //m_driverController.button(6).whileTrue(intakeArm.setAngle(Setpoints.Intake.intakeArmAngleOut));
   // m_driverController.button(7).whileTrue(intakeArm.setAngle(Degrees.of(35)));
    //m_driverController.button(8).whileTrue(intakeArm.setAngle(Degrees.of(55)));
    

    if (Robot.isSimulation()){
     // configureFuelSim();
    }
       String testingMode = "IntakeArm";

    //   if (testingMode.equals("Turret")){
    //     //TODO :: Add commands that control hood angles, Velocity, and pivot with sim
    //       m_driverController.button(1).whileTrue(hoodSubsystem.setAngle(Hood.lowerHoodAngle));//not working
    //       m_driverController.button(2).whileTrue(hoodSubsystem.setAngle(Hood.higherHoodAngle));//not working
    //       m_driverController.button(3).whileTrue(hoodSubsystem.setDutyCycle(1.0));
    //       m_driverController.button(4).whileTrue(hoodSubsystem.setDutyCycle(-1.0));

    //       m_driverController.button(5).whileTrue(turretSubsystem.setAngle(Pivot.leftTurretLimit));
    //       m_driverController.button(6).whileTrue(turretSubsystem.setAngle(Pivot.rightTurretLimit));

    //       m_driverController.button(7).whileTrue(turretSubsystem.set(1.0));//not working
    //       m_driverController.button(8).whileTrue(turretSubsystem.set(-1.0));//not working
          

    // m_driverController.rightTrigger().whileTrue(
    // Commands.either(fullShoot, turretFlywheel.setDutyCycle(-0.4), () -> streamlinedShooting));


    // m_driverController.leftTrigger().whileTrue(
    // Commands.either(fullShoot, kicker.setDutyCycle(-0.9), () -> streamlinedShooting));


    // m_driverController.a().whileTrue(
    // Commands.either(fullShoot, agitator.setDutyCycle(-0.4), () -> streamlinedShooting));


    // m_driverController.b().whileTrue(
    // Commands.either(fullShoot, indexer.setDutyCycle(-0.4), () -> streamlinedShooting));


    // m_driverController.x().onTrue(Commands.runOnce(() -> streamlinedShooting = !streamlinedShooting));


    //   }

    //   if(testingMode.equals("Elevator")){
    //       m_driverController.button(1).whileTrue(climberSubsystem.setHeight((Meters.of(0.8))));
    //       m_driverController.button(2).whileTrue(climberSubsystem.setHeight(Meters.of(0.16)));
    //       m_driverController.button(3).whileTrue(climberSubsystem.set(-0.3));
    //       m_driverController.button(4).whileTrue(climberSubsystem.set(0.3));
    //   }

       if(testingMode.equals("IntakeArm")){//not working
          //  m_driverController.button(1).whileTrue(intakeArm.setAngle(Setpoints.Intake.intakeArmAngleIn));
          //  m_driverController.button(2).whileTrue(intakeArm.setAngle(Setpoints.Intake.intakeArmAngleOut));
          //  //m_driverController.button(3).whileTrue(intakeArm.setAngle(Degrees.of(60)));
          //  //m_driverController.button(4).whileTrue(intakeArm.setAngle(Degrees.of(30)));
          //  m_driverController.button(5).whileTrue(intakeArm.setDutyCycle(0.8));
          //  m_driverController.button(6).whileTrue(intakeArm.setDutyCycle(-0.8));
          
   }




    // var topRightOfTrench = new Pose2d().getTranslation();
    // var bottomLeftOfTrench = new Pose2d().getTranslation();

    // SwerveDrive drive;
    // SwerveInputStream stream;
    // var trenchRight = new Rectangle2d(topRightOfTrench,bottomLeftOfTrench);
    // new Trigger(()->trenchRight.contains(drive.getPose().getTranslation()))
    // .whileTrue(Commands.run(()->driveAngularVelocity.withControllerHeadingAxis(()->Math.cos(Degrees.of(30).in(Radians)),()-> Math.cos(Degrees.of(30).in(Radians))).headingWhile(true))
    // .finallyDo(()->stream.headingWhile(false)));




  }
  //  private void configureFuelSim() {
  //   FuelSim instance = FuelSim.getInstance();
  //   instance.spawnStartingFuel();
  //   instance.registerRobot(
  //       Units.inchesToMeters(30), // Dimensions.FULL_WIDTH.in(Meters)
  //       Units.inchesToMeters(30), // Dimensions.FULL_LENGTH.in(Meters)
  //       Units.inchesToMeters(4), // Dimensions.BUMPER_HEIGHT.in(Meters)
  //       drivebase::getPose,
  //       drivebase::getFieldVelocity);
  //   instance.registerIntake(
  //       -Units.inchesToMeters(15), // -Dimensions.FULL_LENGTH.div(2).in(Meters),
  //       Units.inchesToMeters(15), // Dimensions.FULL_LENGTH.div(2).in(Meters),
  //       -Units.inchesToMeters(
  //           15 + 7), // -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
  //       -Units.inchesToMeters(15), // -Dimensions.FULL_WIDTH.div(2).in(Meters),
  //       () -> true,
  //       () -> turretVisualizer.intakeFuel());

  //   instance.start();
  //   SmartDashboard.putData(
  //       Commands.runOnce(
  //               () -> {
  //                 FuelSim.getInstance().clearFuel();
  //                 FuelSim.getInstance().spawnStartingFuel();
  //               })
  //           .withName("Reset Fuel")
  //           .ignoringDisable(true));
 // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 /*  */ public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected(); 
    // return drivebase.driveToPose(new Pose2d(new Translation2d(16, 2), new Rotation2d())).alongWith(loading.intakeBalls()).withTimeout(6)
    // .andThen(autoSwerveInputStream)
    // .alongWith(new ShootCommand(indexer, kicker, hood, turretFlywheel, TurretConstants.FARShooterGolRPM));
  }

  public Command fullShoot =
turretFlywheel.setDutyCycle(-0.4)
        .alongWith(
            agitator.setDutyCycle(0.4),
            kicker.setDutyCycle(-0.9),
            indexer.setDutyCycle(0.4)
        );

}
