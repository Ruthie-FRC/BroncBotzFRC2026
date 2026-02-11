package frc.robot.systems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretFlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterTargetingSystem {

  double clearance;
  double velocity;
  double initialVelocity;
  double turretPitchAngle;
  public double turretAngle;
  double turretAngleRelative;
  double hubRadius;
  double hubHeight;
  double hubInsideHeight;
  Transform3d ROBOT_TO_TURRET;
  Transform2d ROBOT_TO_TURRET_2D;
  Pose2d goalLocation;
  Translation2d shotVec;

  private final StructPublisher<Pose2d> shotVecPublisher =
    NetworkTableInstance.getDefault()
        .getTable("Drive")
        .getStructTopic("shotVecPublisher", Pose2d.struct)
        .publish();
  private final StructPublisher<Pose2d> newRobotPosePublisher =
    NetworkTableInstance.getDefault()
        .getTable("Drive")
        .getStructTopic("newRobotPosePublisher", Pose2d.struct)
        .publish();

  public ShooterTargetingSystem(Transform3d ROBOT_TO_TURRET) {

      clearance = Units.inchesToMeters(21); // clearance above goal (or smth idk)
      hubRadius = Units.inchesToMeters(24); // inches
      hubHeight = Units.inchesToMeters(72); // inches
      hubInsideHeight = Units.inchesToMeters(48); // inches
      this.ROBOT_TO_TURRET = ROBOT_TO_TURRET;
      ROBOT_TO_TURRET_2D =
          new Transform2d(this.ROBOT_TO_TURRET.getX(), this.ROBOT_TO_TURRET.getY(), new Rotation2d());

  }

   public static Shot findIdealVelocityAndAngle(Pose2d robotPose, ChassisSpeeds robotSpeed) {
    Transform3d ROBOT_TO_TURRET =
        new Transform3d(
            0, // back from robot center
            0, // centered left/right
            0.451739, // up from the floor reference
            new Rotation3d());
    Pose2d goalLocation =
        new Pose2d(
            Units.inchesToMeters(651.2 - 158.6 - 47.0 / 2),
            Units.inchesToMeters(317.7 / 2),
            new Rotation2d());
    double clearance = Units.inchesToMeters(21); // clearance above goal (or smth idk)
    double hubRadius = Units.inchesToMeters(24); // inches
    double hubHeight = Units.inchesToMeters(72); // inches
    double hubInsideHeight = Units.inchesToMeters(48); // inches
    double x1 =
        -1
            * new Pose3d(robotPose)
                .transformBy(ROBOT_TO_TURRET)
                .toPose2d()
                .getTranslation()
                .getDistance(goalLocation.getTranslation());
    // System.out.println("x1 = " + x1);
    double y1 = new Pose3d(robotPose).transformBy(ROBOT_TO_TURRET).getZ();
    double x2 = -1 * hubRadius;
    double y2 = hubHeight + clearance;
    double x3 = 0;
    double y3 = hubInsideHeight;
    System.out.println(
        "x1 = " + x1 + "\n" + "y1 = " + y1 + "\n" + "x2 = " + x2 + "\n" + "y2 = " + y2 + "\n"
            + "x3 = " + x3 + "\n" + "y3 = " + y3);
    double a =
        (y1 * (x2 - x3) + y2 * (-x1 + x3) + y3 * (x1 - x2)) / ((x1 - x2) * (x1 - x3) * (x2 - x3));
    // double a=(y1*(x2-x3)+y2*(-x1+x3)+y3*(x1-x2))/((x1-x2)*(x1-x3)*(x2-x3))
    double b =
        (-y1 * (x2 - x3) * (x2 + x3) + y2 * (x1 + x3) * (x1 - x3) - y3 * (x1 - x2) * (x1 + x2))
            / ((x1 - x2) * (x1 - x3) * (x2 - x3));
    // double c = (y1*x2*x3*(x2-x3)-y2*x1*x3*(x1-x3)+y3*x1*x2*(x1-x2))/((x1-x2)*(x1-x3)*(x2-x3));
    // double xIntercept = (-b-Math.sqrt(b^2-4*a*(c-y1)))/(2*a);

    // Distance FIELD_LENGTH = Inches.of(650.12);
    // Distance FIELD_WIDTH = Inches.of(316.64);
    // ShotData calculatedShot =
    //     iterativeMovingShotFromFunnelClearance(
    //         robotPose,
    //         robotSpeed,
    //         new Translation3d(
    //             Inches.of(650.12).minus(Inches.of(181.56)),
    //             Inches.of(316.64).div(2),
    //             Inches.of(56.4)),
    //         3);

    Angle turretPitchAngle = Angle.ofBaseUnits(Math.atan(2 * a * x1 + b), Radians);
    // turretPitchAngle = calculatedShot.getHoodAngle().magnitude();
    // initialVelocity = calculatedShot.getExitVelocity().magnitude();
    // System.out.println(Math.atan(2 * a * x1 + b));
    // System.out.println(Math.sqrt((-9.81) / (2 * a * Math.pow(Math.cos(turretPitchAngle), 2))));
    // System.out.println(calculatedShot.getTarget());
    // velocity = (xIntercept-x1)/Math.cos(turretPitchAngle);
    LinearVelocity initialVelocity =
        LinearVelocity.ofBaseUnits(
            Math.sqrt(
                (-9.81) / (2 * a * Math.pow(Math.cos(turretPitchAngle.baseUnitMagnitude()), 2))),
            MetersPerSecond);

    System.out.println(
        "a = "
            + a
            + "\n"
            + "b = "
            + b
            + "\n"
            + "turretPitchAngle = "
            + turretPitchAngle
            + "\n"
            + "initialVelocity = "
            + initialVelocity);

    return new Shot(initialVelocity, turretPitchAngle, Angle.ofBaseUnits(0, Radians));
  }


  public static Time calculateTimeOfFlight(
      LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
    double vel = exitVelocity.in(MetersPerSecond);
    double angle = hoodAngle.in(Radians);
    double dist = distance.in(Meters);
    return Seconds.of(dist / (vel * Math.cos(angle)));
  }

  public static Translation2d getTargetVector(Pose2d robotPose) {
    Pose2d goalLocation =
        new Pose2d(
            Units.inchesToMeters(651.2 - 158.6 - 47.0 / 2),
            Units.inchesToMeters(317.7 / 2),
            new Rotation2d());
    Translation2d targetVec = goalLocation.getTranslation().minus(robotPose.getTranslation());
    return targetVec;
    // double dist = targetVec.getNorm();
  }

  public static Pose2d getNewRobotPose(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds, Time timeOfFlight) {
    Pose2d newRobotPose =
        robotPose.transformBy(
            new Transform2d(
                getRobotVelocityVector(chassisSpeeds).times(-1 * timeOfFlight.baseUnitMagnitude()),
                new Rotation2d()));
    return newRobotPose;
  }

  public static Translation2d getRobotVelocityVector(ChassisSpeeds chassisSpeeds) {
    Translation2d robotVelVec =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    return robotVelVec;
  }

  public static Pose2d getNewGoalLocation(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds, Time timeOfFlight) {
    Pose2d goalLocation =
        new Pose2d(
            Units.inchesToMeters(651.2 - 158.6 - 47.0 / 2),
            Units.inchesToMeters(317.7 / 2),
            new Rotation2d());
    Pose2d newGoalLocation =
        goalLocation.transformBy(
            new Transform2d(
                getRobotVelocityVector(chassisSpeeds).times(-1 * timeOfFlight.baseUnitMagnitude()),
                new Rotation2d()));
    return newGoalLocation;
  }

  public static Translation2d getShotVector(Pose2d newGoalLocation, Pose2d robotPose) {
    Translation2d shotVec = newGoalLocation.getTranslation().minus(robotPose.getTranslation());
    return shotVec;
  }

  //   public double getVelocity() {
  //     return velocity;
  //   }

  //   public double getTurretPitchAngle() {
  //     return turretPitchAngle;
  //   }

  public static Shot getShotData(Pose2d oldRobotPose, ChassisSpeeds chassisSpeeds, int iterations) {
    Transform3d ROBOT_TO_TURRET =
        new Transform3d(
            0, // back from robot center
            0, // centered left/right
            0.451739, // up from the floor reference
            new Rotation3d());
    // 1. LATENCY COMP
    System.out.println("shooterAimer time = " + RobotContainer.timerThing.get());
    double latency = 0.37; // Tuned constant
    Pose2d robotPose =
        oldRobotPose
            // .transformBy(ROBOT_TO_TURRET.)
            .plus(
            new Transform2d(
                    chassisSpeeds.vxMetersPerSecond
                        * ((chassisSpeeds.vxMetersPerSecond < 0) ? -1 : 1),
                    chassisSpeeds.vyMetersPerSecond,
                    new Rotation2d())
                .times(-1 * latency));
    System.out.println(
        "chassisSpeeds X thing = "
            + (chassisSpeeds.vxMetersPerSecond * ((chassisSpeeds.vxMetersPerSecond < 0) ? -1 : 1)));
    System.out.println("chassisSpeeds Y thing = " + (chassisSpeeds.vyMetersPerSecond));

    Shot initialCalcShot = findIdealVelocityAndAngle(robotPose, chassisSpeeds);
    Translation2d targetVec = getTargetVector(robotPose);

    Time timeOfFlight =
        calculateTimeOfFlight(
            initialCalcShot.getVelocity(),
            initialCalcShot.getPitchAngle(),
            Distance.ofBaseUnits(targetVec.getNorm(), Meters));

    Pose2d newGoalLocation = getNewGoalLocation(robotPose, chassisSpeeds, timeOfFlight);
    Pose2d newRobotPose = getNewRobotPose(robotPose, chassisSpeeds, timeOfFlight);
    Translation2d shotVec = getShotVector(newGoalLocation, robotPose);

    for (int i = 0; i < iterations; i++) {
      newGoalLocation = getNewGoalLocation(robotPose, chassisSpeeds, timeOfFlight);
      newRobotPose = getNewRobotPose(robotPose, chassisSpeeds, timeOfFlight);

      targetVec = getTargetVector(robotPose);
      timeOfFlight =
          calculateTimeOfFlight(
              initialCalcShot.getVelocity(),
              initialCalcShot.getPitchAngle(),
              Distance.ofBaseUnits(targetVec.getNorm(), Meters));

      shotVec = getShotVector(newGoalLocation, robotPose);
    }

    Angle turretAngle = Angle.ofBaseUnits(shotVec.getAngle().getRadians(), Radians);
    Shot newCalcShot = findIdealVelocityAndAngle(newRobotPose, chassisSpeeds);
    Shot shot = new Shot(newCalcShot.getVelocity(), newCalcShot.getPitchAngle(), turretAngle);

    return shot;
  }

 


  public record Shot(double exitVelocity, double hoodAngle, double target) {
    public Shot(LinearVelocity exitVelocity, Angle hoodAngle, Angle target) {
      this(exitVelocity.in(MetersPerSecond), hoodAngle.in(Radians), target.in(Radians));
    }

    public LinearVelocity getVelocity() {
      return MetersPerSecond.of(this.exitVelocity);
    }

    public Angle getPitchAngle() {
      return Radians.of(this.hoodAngle);
    }

    public Angle getAngle() {
      return Radians.of(this.target);
    }
}
}
