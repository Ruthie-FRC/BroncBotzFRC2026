package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Setpoints.Turret.Hood;
import frc.robot.Setpoints.Turret.Pivot;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretFlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import swervelib.SwerveInputStream;
import yams.units.YUnits;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
  private final IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();

  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  private final TurretFlywheelSubsystem turretFlywheelSubsystem = new TurretFlywheelSubsystem();

  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final AgitatorSubsystem agitatorSubsystem = new AgitatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    defaultCommands();
  }



  public void defaultCommands() {
    drivebase.setDefaultCommand(
        !RobotBase.isSimulation()
            ? driveFieldOrientedAngularVelocity
            : driveFieldOrientedDirectAngleKeyboard);
    agitatorSubsystem.setDefaultCommand(agitatorSubsystem.setDutyCycle(0));
    indexerSubsystem.setDefaultCommand(indexerSubsystem.setDutyCycle(0));

    turretFlywheelSubsystem.setDefaultCommand(
        turretFlywheelSubsystem.setVelocity(YUnits.SandwichPerSecond.of(0)));

    turretSubsystem.setDefaultCommand(
        turretSubsystem.setAngle(Pivot.startTurretAngle));
    hoodSubsystem.setDefaultCommand(hoodSubsystem.setAngle(Hood.startHoodAngle));

    intakeArmSubsystem.setDefaultCommand(
        intakeArmSubsystem.setAngle(Setpoints.Intake.intakeArmStartAngle));
    climberSubsystem.setDefaultCommand(climberSubsystem.setHeight(Setpoints.Climber.startHeight));
  }

  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> m_driverController.getLeftY() * -1,
              () -> m_driverController.getLeftX() * -1)
          .withControllerRotationAxis(m_driverController::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
          .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  // simulation stuff

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

  Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngle);

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

      String testingMode = "Turret";

      if (testingMode.equals("Turret")){
        //TODO :: Add commands that control hood angles, Velocity, and pivot with sim
          m_driverController.button(1).whileTrue(hoodSubsystem.setAngle(Hood.lowerHoodAngle));//not working
          m_driverController.button(2).whileTrue(hoodSubsystem.setAngle(Hood.higherHoodAngle));//not working
          m_driverController.button(3).whileTrue(hoodSubsystem.setDutyCycle(1.0));
          m_driverController.button(4).whileTrue(hoodSubsystem.setDutyCycle(-1.0));

          m_driverController.button(5).whileTrue(turretSubsystem.setAngle(Pivot.leftTurretLimit));
          m_driverController.button(6).whileTrue(turretSubsystem.setAngle(Pivot.rightTurretLimit));

          m_driverController.button(7).whileTrue(turretSubsystem.set(1.0));//not working
          m_driverController.button(8).whileTrue(turretSubsystem.set(-1.0));//not working
          


      }

      if(testingMode.equals("Elevator")){
          m_driverController.button(1).whileTrue(climberSubsystem.setHeight((Meters.of(0.8))));
          m_driverController.button(2).whileTrue(climberSubsystem.setHeight(Meters.of(0.16)));
          m_driverController.button(3).whileTrue(climberSubsystem.set(-0.3));
          m_driverController.button(4).whileTrue(climberSubsystem.set(0.3));
      }

      if(testingMode.equals("IntakeArm")){//not working
          m_driverController.button(1).whileTrue(intakeArmSubsystem.setAngle(Setpoints.Intake.intakeArmAngleIn));
          m_driverController.button(2).whileTrue(intakeArmSubsystem.setAngle(Setpoints.Intake.intakeArmAngleOut));
          m_driverController.button(3).whileTrue(intakeArmSubsystem.setDutyCycle(0.8));
          m_driverController.button(4).whileTrue(intakeArmSubsystem.setDutyCycle(-0.8));
          
      }









  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }


}
