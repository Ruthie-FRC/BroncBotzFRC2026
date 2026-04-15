package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import java.util.List;
import java.util.Optional;

//import com.google.flatbuffers.Constants;
import frc.robot.Constants;

import frc.robot.Setpoints;
import frc.robot.subsystems.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;


public class ShootKickIndexCommand extends Command {

    private record RecordedShot(Distance distance, AngularVelocity shooterSpeed, //Angle hoodAngle, 
    Time tof) {
        public Pair<Double, Double> getRPM() {
            return Pair.of(distance.in(Meters), shooterSpeed.in(RPM));
        }

        public Pair<Double, Double> getTOF() {
            return Pair.of(distance.in(Meters), tof.in(Second));
        }

        // public Pair<Double, Double> getHoodAngle() {
        //     return Pair.of(distance.in(Meters), hoodAngle.in(Degrees));
        // }
    }

    private final FlywheelSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;
    private final AgitatorSubsystem agitator;
    private final Optional<SwerveSubsystem> swerve;
    private final HoodSubsystem hood;//94 rpm
    private final AngularVelocity goalRPM;   // <-- parameter stored here
    //private final Angle goalDegree;

    private final Debouncer shootDebounce1 = new Debouncer(0.3, DebounceType.kFalling);

    private final List<RecordedShot> shots = List.of(
            // TUNE HERE
            new RecordedShot(Inches.of(196), RPM.of(3000),  Second.of(1)),
            new RecordedShot(Inches.of(165), RPM.of(2800),Second.of(1)),
            new RecordedShot(Inches.of(140), RPM.of(2600), Second.of(1)),
             new RecordedShot(Inches.of(116), RPM.of(2400),  Second.of(1)),
            new RecordedShot(Inches.of(93), RPM.of(2200),Second.of(1)),
            new RecordedShot(Inches.of(208), RPM.of(3200), Second.of(1)),
             new RecordedShot(Inches.of(202), RPM.of(3150),  Second.of(1)),
            new RecordedShot(Inches.of(191), RPM.of(3100),Second.of(1)),
            new RecordedShot(Inches.of(191), RPM.of(3050), Second.of(1)),
            new RecordedShot(Inches.of(132), RPM.of(2500),  Second.of(1)),
            new RecordedShot(Inches.of(115), RPM.of(2400),Second.of(1)),
            new RecordedShot(Inches.of(105), RPM.of(2400), Second.of(1)),
             new RecordedShot(Inches.of(97), RPM.of(2200),  Second.of(1)),
            new RecordedShot(Inches.of(207), RPM.of(3100), Second.of(1)),
             new RecordedShot(Inches.of(174), RPM.of(2800),  Second.of(1)),
            new RecordedShot(Inches.of(162), RPM.of(2700),Second.of(1)),
            new RecordedShot(Inches.of(153), RPM.of(2600), Second.of(1)),
            new RecordedShot(Inches.of(133), RPM.of(2500), Second.of(1))

    );
    private final InterpolatingDoubleTreeMap calculatedGoalRPM = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap calculatedTOF = new InterpolatingDoubleTreeMap();
   // private final InterpolatingDoubleTreeMap calculatedHoodAngle = new InterpolatingDoubleTreeMap();

    public ShootKickIndexCommand(
            FlywheelSubsystem shooter,
            KickerSubsystem kicker,
            IndexerSubsystem indexer,
            AgitatorSubsystem agitator,
            HoodSubsystem hood,
            AngularVelocity goalRPM1
           // Angle goalDegree1   // <-- parameter passed in
    ) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.agitator = agitator;
        this.hood = hood;
        this.swerve = Optional.empty();
        
        this.goalRPM = goalRPM1;   // <-- store parameter
        //this.goalDegree = goalDegree1;

        addRequirements(this.shooter, this.kicker, this.indexer, this.agitator, this.hood);
    }

    public ShootKickIndexCommand(
            FlywheelSubsystem shooter,
            KickerSubsystem kicker,
            IndexerSubsystem indexer,
            AgitatorSubsystem agitator,
            HoodSubsystem hood,
            SwerveSubsystem swerve
    ) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.agitator = agitator;
        this.hood = hood;
        this.swerve = Optional.of(swerve);
        goalRPM = RPM.zero();
        //goalDegree = Degrees.zero();

        for (var shot : shots) {
            calculatedGoalRPM.put(shot.distance.in(Meters), shot.shooterSpeed.in(RPM));
            calculatedTOF.put(shot.distance.in(Meters), shot.tof.in(Second));
           // calculatedHoodAngle.put(shot.distance.in(Meters), shot.hoodAngle.in(Degrees));
        }
        addRequirements(this.shooter, this.kicker, this.indexer, this.hood);
    }

    

    @Override
    public void initialize() {
        // Spin up shooter to the passed RPM
        kicker.setVelocitySetpoint(RPM.of(Constants.KickerConstants.kickerRPM));
        hood.setAngleSetpoint(Setpoints.Intake.hoodUpAngle);
        if (swerve.isEmpty()) {
            shooter.setVelocitySetpoint(goalRPM);
            hood.setAngleSetpoint(Setpoints.Intake.hoodUpAngle);
        }
        
    }

    @Override
    public void execute() {

        AngularVelocity goalRPM1 = goalRPM;
        //Angle goalDegree1 = goalDegree;
        if (swerve.isPresent()) {
            goalRPM1 = RPM.of(calculatedGoalRPM.get(swerve.get().distanceToHub()));
            //goalDegree1 = Degrees.of(calculatedHoodAngle.get(swerve.get().distanceToHub()));
        }

        shooter.setVelocitySetpoint(goalRPM1);
        hood.setAngleSetpoint(Setpoints.Intake.hoodUpAngle);

        AngularVelocity shooterRPM = shooter.getRPM();

        boolean shooterReady = shootDebounce1.calculate(
                shooterRPM.isNear(
                        goalRPM1,
                        RPM.of(150)// tolerance
                )

        );
        kicker.setVelocitySetpoint(RPM.of(Constants.KickerConstants.kickerRPM));

       agitator.setDutyCycleSetpoint(Constants.AgitatorConstants.AgitatorSpeed);//RPM.of(1135)

        if (shooterReady) {

            
            indexer.setVelocitySetpoint(RPM.of(Constants.IndexerConstants.indexerRPM));
        } else {
            indexer.setDutyCycleSetpoint(Constants.IndexerConstants.indexerInverse);;
        }


    }

    @Override
    public void end(boolean interrupted) {
        shooter.setDutyCycleSetpoint(0);
        kicker.setDutycycleSetpoint(0);
        indexer.setDutyCycleSetpoint(0);
        agitator.setDutyCycleSetpoint(0);
        hood.setDutyCycleSetpoint(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}