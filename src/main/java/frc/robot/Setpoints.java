package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import yams.units.YUnits;

import static edu.wpi.first.units.Units.*;

public class Setpoints {
    public static class Shooter {
        public static final AngularVelocity hubRPM = RPM.of(3000);
    }

    public static class Intake {
        public static final AngularVelocity intakeRollerRPM = RPM.of(-1000);
        public static final AngularVelocity outtakeRollerRPM = RPM.of(1000);
        public static final Angle           intakeArmStartAngle = Degrees.of(0); // change back to 45
        public static final Angle           intakeArmAngleUp    = Degrees.of(55);
        public static final AngularVelocity agitatorRPMout      = RPM.of(-500);
        public static final AngularVelocity agitatorRPMin      = RPM.of(500);
        public static Angle                 intakeArmAngleDown = Degrees.of(0);

      public static Angle masterIntakeAbsoluteEncoderOffset = Degrees.of(0);
      public static Angle followerIntakeAbsoluteEncoderOffset = Degrees.of(0);

    }

    public static class Indexer {
        // indexSubsystem and agitator
        public static final AngularVelocity startFlywheelAngle = YUnits.RPY.of(0);
    }


    public static class Hood {
        public static final Angle hubDegree = Degrees.of(15);
        public static final Angle startHoodAngle = Degrees.of(0);
        public static final Angle lowerHoodAngle = Degrees.of(0);
        public static final Angle higherHoodAngle = Degrees.of(25);
    }

    

}
