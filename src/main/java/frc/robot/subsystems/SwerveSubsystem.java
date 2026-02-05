package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlignmentConstants.DriveToPose;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  File directory = new File(Filesystem.getDeployDirectory(), "swerve");

  static SwerveDrive swerveDrive;
  Limelight limelight;
  LimelightPoseEstimator limelightPoseEstimator;

  public SwerveSubsystem() {
    // error catching
    try {
      swerveDrive =
          new SwerveParser(directory)
              .createSwerveDrive(
                  Constants.maxSpeed,
                  new Pose2d(
                      new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    setupPathPlanner();
    setupLimelight();
  }

  public void setupLimelight() {
    swerveDrive.stopOdometryThread();
    limelight = new Limelight("limelight");
    limelight
        .getSettings()
        .withPipelineIndex(0)
        .withCameraOffset(
            new Pose3d(
                Units.inchesToMeters(12),
                Units.inchesToMeters(12),
                Units.inchesToMeters(10.5),
                new Rotation3d(0, 0, Units.degreesToRadians(45))))
        .withAprilTagIdFilter(List.of(17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11))
        .save();
    limelightPoseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
  }

  /**
   * Returns a Command that tells the robot to drive forward until the command ends.
   *
   * @return a Command that tells the robot to drive forward until the command ends
   */
  public Command driveForward() {
    return run(() -> {
          swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
        })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  private int outofAreaReading = 0;
  private boolean initialReading = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelight
        .getSettings()
        .withRobotOrientation(
            new Orientation3d(
                new Rotation3d(swerveDrive.getOdometryHeading().rotateBy(Rotation2d.kZero)),
                new AngularVelocity3d(
                    DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0))))
        .save();
    Optional<PoseEstimate> poseEstimates = limelightPoseEstimator.getPoseEstimate();
    Optional<LimelightResults> results = limelight.getLatestResults();
    if (results.isPresent() /* && poseEstimates.isPresent()*/) {
      LimelightResults result = results.get();
      PoseEstimate poseEstimate = poseEstimates.get();
      SmartDashboard.putNumber("Avg Tag Ambiguity", poseEstimate.getAvgTagAmbiguity());
      SmartDashboard.putNumber("Min Tag Ambiguity", poseEstimate.getMinTagAmbiguity());
      SmartDashboard.putNumber("Max Tag Ambiguity", poseEstimate.getMaxTagAmbiguity());
      SmartDashboard.putNumber("Avg Distance", poseEstimate.avgTagDist);
      SmartDashboard.putNumber("Avg Tag Area", poseEstimate.avgTagArea);
      SmartDashboard.putNumber("Odom Pose/x", swerveDrive.getPose().getX());
      SmartDashboard.putNumber("Odom Pose/y", swerveDrive.getPose().getY());
      SmartDashboard.putNumber(
          "Odom Pose/degrees", swerveDrive.getPose().getRotation().getDegrees());
      SmartDashboard.putNumber("Limelight Pose/x", poseEstimate.pose.getX());
      SmartDashboard.putNumber("Limelight Pose/y", poseEstimate.pose.getY());
      SmartDashboard.putNumber(
          "Limelight Pose/degrees", poseEstimate.pose.toPose2d().getRotation().getDegrees());
      if (result.valid) {
        // Pose2d estimatorPose = poseEstimate.pose.toPose2d();
        Pose2d usefulPose = result.getBotPose2d(Alliance.Blue);
        double distanceToPose =
            usefulPose.getTranslation().getDistance(swerveDrive.getPose().getTranslation());
        if (distanceToPose < 0.5
            || (outofAreaReading > 10)
            || (outofAreaReading > 10 && !initialReading)) {
          if (!initialReading) {
            initialReading = true;
          }
          outofAreaReading = 0;
          // System.out.println(usefulPose.toString());
          swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
          // System.out.println(result.timestamp_LIMELIGHT_publish);
          // System.out.println(result.timestamp_RIOFPGA_capture);
          swerveDrive.addVisionMeasurement(usefulPose, result.timestamp_RIOFPGA_capture);
        } else {
          outofAreaReading += 1;
        }
        //        swerveDrive.addVisionMeasurement(estimatorPose, poseEstimate.timestampSeconds);
      }
    }
  }

  public Rotation2d getOdometryHeading() {
    return swerveDrive.getOdometryHeading();
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
   * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls
   * to resetOdometry().
   *
   * @return The yaw angle
   */
  public static Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public static Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Rotation2d getRotation() {
    return swerveDrive.getYaw();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally
          // outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive
              // trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
              ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
          );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPose(Supplier<Pose2d> pose) {
    double tooCloseMeters =
        0.5; // If the bot is too close by this much it needs to drive back a little bit.
    return defer(() -> driveToPose(pose.get()));
  }

  public Command pathndToPose(Supplier<Pose2d> pose) {
    return defer(
        () -> {
          PathConstraints constraints =
              new PathConstraints(
                  DriveToPose.maximumVelocityMetersPerSecond,
                  DriveToPose.maximumAccelerationMetersPerSecondSquared,
                  Degrees.of(DriveToPose.maximumAngularVelocityDegreesPerSecond)
                      .per(Second)
                      .in(RadiansPerSecond),
                  Units.degreesToRadians(
                      DriveToPose.maximumAngularAccelerationDegreesPerSecondSquared));
          // Since AutoBuilder is configured, we can use it to build pathfinding commands
          return AutoBuilder.pathfindToPose(
              pose.get(),
              constraints,
              edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
              );
        });
  }

  public void stopDriving() {
    swerveDrive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  public Command stopDrivingCommand() {
    return runOnce(() -> stopDriving());
  }

  public Command driveToPose(Pose2d pose) {
    stopDriving();
    boolean useSetpointGenerator = false;
    boolean useProfiledController = true;
    boolean resetBeforehand = true;
    Supplier<ChassisSpeeds> robotRelativeSpeeds;
    BooleanSupplier atTargetPose;

    if (resetBeforehand) {
      DriveToPose.driveController.getXController().reset();
      DriveToPose.driveController.getYController().reset();
      DriveToPose.driveController.getThetaController().reset(getPose().getRotation().getRadians());

      if (useProfiledController) {
        DriveToPose.profiledDriveController.reset(getSwerveDrive().getPose());
      }
    }
    if (useProfiledController) {
      atTargetPose = DriveToPose.profiledDriveController::atReference;
      robotRelativeSpeeds =
          () ->
              DriveToPose.profiledDriveController.calculate(getPose(), pose, 0, pose.getRotation());
    } else {
      atTargetPose = DriveToPose.driveController::atReference;
      robotRelativeSpeeds =
          () -> DriveToPose.driveController.calculate(getPose(), pose, 0, pose.getRotation());
    }

    if (useSetpointGenerator) {
      try {

        return driveWithSetpointGenerator(robotRelativeSpeeds)
            .until(atTargetPose)
            .finallyDo(this::stopDriving);
      } catch (Exception ignored) {
        DriverStation.reportWarning("Could not use setpoint generator with drive to pose.", false);
      }
    }

    return run(() -> swerveDrive.drive(robotRelativeSpeeds.get()))
        .until(atTargetPose)
        .finallyDo(this::stopDriving);
    /*
    // Create the constraints to use while pathfinding
        */
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   * @throws org.json.simple.parser.ParseException
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    try {
      SwerveSetpointGenerator setpointGenerator =
          new SwerveSetpointGenerator(
              RobotConfig.fromGUISettings(), swerveDrive.getMaximumChassisAngularVelocity());

      AtomicReference<SwerveSetpoint> prevSetpoint =
          new AtomicReference<>(
              new SwerveSetpoint(
                  swerveDrive.getRobotVelocity(),
                  swerveDrive.getStates(),
                  DriveFeedforwards.zeros(swerveDrive.getModules().length)));
      AtomicReference<Double> previousTime = new AtomicReference<>();

      return startRun(
          () -> previousTime.set(Timer.getFPGATimestamp()),
          () -> {
            double newTime = Timer.getFPGATimestamp();
            SwerveSetpoint newSetpoint =
                setpointGenerator.generateSetpoint(
                    prevSetpoint.get(),
                    robotRelativeChassisSpeed.get(),
                    newTime - previousTime.get());
            swerveDrive.drive(
                newSetpoint.robotRelativeSpeeds(),
                newSetpoint.moduleStates(),
                newSetpoint.feedforwards().linearForces());
            prevSetpoint.set(newSetpoint);
            previousTime.set(newTime);
          });
    } catch (Exception e) {
      return null;
    }
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(
      Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(
          () -> {
            return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
          });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  public Command driveForwards() {
    return run(() -> {
          swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
        })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command driveBackwards() {
    return run(() -> {
          swerveDrive.drive(new Translation2d(-1, 0), 0, false, false);
        })
        .finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
  }

  public Command lockPos() {
    return run(swerveDrive::lockPose);
  }

  public Command drive(Supplier<ChassisSpeeds> driveAngularVelocity) {
    return run(
        () -> {
          swerveDrive.drive(driveAngularVelocity.get());
        });
  }

  public Command rotateToHeading(Rotation2d rotation2d) {
    return run(
        () ->
            swerveDrive.drive(
                new Translation2d(0, 0),
                swerveDrive
                    .getSwerveController()
                    .headingCalculate(
                        getHeading().getRadians(),
                        getHeading().getRadians() - rotation2d.getRadians()),
                false,
                true));
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
}
