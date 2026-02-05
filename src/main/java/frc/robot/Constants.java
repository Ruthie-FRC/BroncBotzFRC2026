package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import yams.gearing.MechanismGearing;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double maxSpeed = 4.5; // meters per second
  public static boolean disableHAL = false;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
  }

  public static class CanIDConstants {

    public static final int climberCanID = 11;

    public static final int intakeRollerID = 20;
    public static final int turretID = 30;
    public static final int hoodID = 31;
    public static final int agitatorID = 32;
    public static final int indexerID = 33;
    public static final int indexerflywheelID = 34;
    public static final int turretFlywheelID = 35;

    public static final int intakeArmID = 40;
  }

  public static class Agitator {
    public static double kP = 0;
    public static double kD = 0;
    public static double kI = 0;

    public static double kS = 0.27937;
    public static double kV = 0.089836;
    public static double kA = 0.014557;
  }

  public static class IntakeConstants {

    public static final String motorTelemetryName = "ExponentiallyProfiledArmMotor";
    public static final String mechTelemetryName = "ExponentiallyProfiledArm";

    public static final MechanismGearing gearing = new MechanismGearing(7);
    public static final Mass weight = Pounds.of(10);
    public static final Distance length = Feet.of(2);
    /*
     * Using the protractor, where 0deg on the protractor is when the arm is parallel to the ground,
     * you can measure where the starting angle should be.
     */
    public static final Angle startingAngle = Degrees.of(30);
    /*
     * To find these limits measure the starting angle relative to when the arm is parallel to the ground using a protractor.
     */
    public static final Angle softLowerLimit = Degrees.of(-10);
    public static final Angle softUpperLimit = Degrees.of(125);
    /*
     * These are the real "limits" of the robot shown in simulation.
     */
    public static final Angle hardLowerLimit = Degrees.of(-30);
    public static final Angle hardUpperLimit = Degrees.of(145);
    // Intake
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;

    public static double kPSim = 1;
    public static double kISim = 0;
    public static double kDSim = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    public static final double kSSim = 0;
    public static final double kVSim = 0;
    public static final double kASim = 0;
    public static final double kGSim = 0;

    public static final double tolerance = 2;
  }

  public static class IntakeRollerConstants {
    public static double kP = 0.2;
    public static double kI = 0;
    public static double kD = 0;

    public static double kPSim = 0.2;
    public static double kISim = 0;
    public static double kDSim = 0;

    public static double IntakeRollerOuttakeSpeeds = -.5;
    public static double IntakeRollerIntakeSpeeds = .5;
    public static double IntakeRollerHoldSpeed = 0.3;

    public static double kWristMomentOfInertia = 0.5;
    public static double kWristGearRatio = 3.0;
  }

  public static class TurretConstants {

    public static double wheelDiameter = 0;
    public static Pose3d cameraOffsetFromRobotCenter =
        new Pose3d(new Translation3d(), new Rotation3d());
    public static Translation3d turretPivotCenterFromRobotCenter =
        new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0));

    public static class PivotConstants {

      public static double kP = 1;
      public static double kD = 0;
      public static double kI = 0;

      public static double kPSim = 1;
      public static double kISim = 0;
      public static double kDSim = 0;

      public static final double kS = 0;
      public static final double kV = 0;
      public static final double kA = 0;
      public static final double kG = 0;

      public static final double kSSim = 0;
      public static final double kVSim = 0;
      public static final double kASim = 0;
      public static final double kGSim = 0;
      public static final Angle tolerance = Degrees.of(2);
      public static Angle softLimitMin = Degrees.of(-30);
    }
  }

  public static class ClimberConstants {
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kPSim = 1;
    public static final double kISim = 0;
    public static final double kDSim = 0;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    public static final double kSSim = 0;
    public static final double kVSim = 0;
    public static final double kASim = 0;
    public static final double kGSim = 0;

    public static final double mass = 15;
  }
}
