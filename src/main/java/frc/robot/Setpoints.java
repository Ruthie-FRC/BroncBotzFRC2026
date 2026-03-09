package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import yams.units.YUnits;

import static edu.wpi.first.units.Units.*;

public class Setpoints {
    public static class Shooter {
        public static final AngularVelocity hubRPM = RPM.of(3000);
    }

    public static class Intake {
        public static final AngularVelocity intakeRollerRPM = RPM.of(-1000);
        public static final AngularVelocity outtakeRollerRPM = RPM.of(1000);
        public static final Angle intakeArmStartAngle = Degrees.of(45);
        public static final Angle intakeArmAngleIn = Degrees.of(10);
        public static Angle intakeArmAngleOut = Degrees.of(50);

    }

    public static class Indexer {
        // indexSubsystem and agitator
        public static final AngularVelocity startFlywheelAngle = YUnits.RPY.of(0);
    }

    public static class Turret {

        public static class Hood {
            public static final Angle startHoodAngle = Degrees.of(30);
            public static final Angle lowerHoodAngle = Degrees.of(20);
            public static final Angle higherHoodAngle = Degrees.of(45);
        }

    }

}
