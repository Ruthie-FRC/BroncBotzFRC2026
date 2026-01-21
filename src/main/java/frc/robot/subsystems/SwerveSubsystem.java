// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightResults;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;

import java.io.File;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive             swerveDrive;
  Limelight               limelight;  
  LimelightPoseEstimator  limelightPoseEstimator;
  
  

  public SwerveSubsystem() {
//error catching
 try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                                Meter.of(4)),
                                                                                      Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    
    setupPathPlanner();
    setupLimelight();
  }

  public void setupLimelight(){
    swerveDrive.stopOdometryThread();
    limelight.getSettings()
             .withPipelineIndex(0)
             .withCameraOffset(new Pose3d(Units.inchesToMeters(12),
                                          Units.inchesToMeters(12),
                                          Units.inchesToMeters(10.5),
                                          new Rotation3d(0, 0, Units.degreesToRadians(45))))
             .withArilTagIdFilter(List.of(17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0))
             .save();
    limelightPoseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
  }
    /**
   * Returns a Command that tells the robot to drive forward until the command ends.
   *
   * @return a Command that tells the robot to drive forward until the command ends
   */
  public Command driveForward()
  {
    return run(() -> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> swerveDrive.drive(new Translation2d(0, 0), 0, false, false));
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

  private int     outofAreaReading = 0;
  private boolean initialReading = false;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelight.getSettings()
             .withRobotOrientation(new Orientation3d(new Rotation3d(swerveDrive.getOdometryHeading()
                                                                               .rotateBy(Rotation2d.kZero)),
                                                     new AngularVelocity3d(DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(0),
                                                                           DegreesPerSecond.of(0))))
             .save();
    Optional<PoseEstimate>     poseEstimates = limelightPoseEstimator.getPoseEstimate();
    Optional<LimelightResults> results       = limelight.getLatestResults();
    if (results.isPresent()/* && poseEstimates.isPresent()*/)
    {
      LimelightResults result       = results.get();
      PoseEstimate     poseEstimate = poseEstimates.get();
      SmartDashboard.putNumber("Avg Tag Ambiguity", poseEstimate.getAvgTagAmbiguity());
      SmartDashboard.putNumber("Min Tag Ambiguity", poseEstimate.getMinTagAmbiguity());
      SmartDashboard.putNumber("Max Tag Ambiguity", poseEstimate.getMaxTagAmbiguity());
      SmartDashboard.putNumber("Avg Distance", poseEstimate.avgTagDist);
      SmartDashboard.putNumber("Avg Tag Area", poseEstimate.avgTagArea);
      SmartDashboard.putNumber("Odom Pose/x", swerveDrive.getPose().getX());
      SmartDashboard.putNumber("Odom Pose/y", swerveDrive.getPose().getY());
      SmartDashboard.putNumber("Odom Pose/degrees", swerveDrive.getPose().getRotation().getDegrees());
      SmartDashboard.putNumber("Limelight Pose/x", poseEstimate.pose.getX());
      SmartDashboard.putNumber("Limelight Pose/y", poseEstimate.pose.getY());
      SmartDashboard.putNumber("Limelight Pose/degrees", poseEstimate.pose.toPose2d().getRotation().getDegrees());
      if (result.valid)
      {
        // Pose2d estimatorPose = poseEstimate.pose.toPose2d();
        Pose2d usefulPose     = result.getBotPose2d(Alliance.Blue);
        double distanceToPose = usefulPose.getTranslation().getDistance(swerveDrive.getPose().getTranslation());
        if (distanceToPose < 0.5 || (outofAreaReading > 10) || (outofAreaReading > 10 && !initialReading))
        {
          if (!initialReading)
          {
            initialReading = true;
          }
          outofAreaReading = 0;
          // System.out.println(usefulPose.toString());
          swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 0.022));
          // System.out.println(result.timestamp_LIMELIGHT_publish);
          // System.out.println(result.timestamp_RIOFPGA_capture);
          swerveDrive.addVisionMeasurement(usefulPose, Timer.getTimestamp());
        } else
        {
          outofAreaReading += 1;
        }
//        swerveDrive.addVisionMeasurement(estimatorPose, poseEstimate.timestampSeconds);
      }

    }

  }

  public Rotation2d getOdometryHeading(){
    return swerveDrive.getOdometryHeading();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }




  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }


public void driveFieldOriented(ChassisSpeeds velocity){
  swerveDrive.driveFieldOriented(velocity);
}

public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
  return run(() -> {
    swerveDrive.driveFieldOriented(velocity.get());
  });
}
public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
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
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
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
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
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
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
}