package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import java.util.List;
import java.util.Optional;

import frc.robot.subsystems.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;


public class ShootKickIndexCommand extends Command {

    private record RecordedShot(Distance distance, AngularVelocity shooterSpeed, Time tof) {
        public Pair<Double, Double> getRPM() {
            return Pair.of(distance.in(Meters), shooterSpeed.in(RPM));
        }

        public Pair<Double, Double> getTOF() {
            return Pair.of(distance.in(Meters), tof.in(Second));
        }
    }

    private final FlywheelSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;
    private final AgitatorSubsystem agitatorSubsystem;
    private final Optional<SwerveSubsystem> swerve;

    private final AngularVelocity goalRPM;   // <-- parameter stored here

    private final Debouncer shootDebounce1 = new Debouncer(0.3, DebounceType.kFalling);

    private final List<RecordedShot> shots = List.of(
            // TUNE HERE
            new RecordedShot(Meters.of(1), RPM.of(4000), Second.of(1)),
            new RecordedShot(Meters.of(1), RPM.of(4000), Second.of(1))

    );
    private final InterpolatingDoubleTreeMap calculatedGoalRPM = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap calculatedTOF = new InterpolatingDoubleTreeMap();

    public ShootKickIndexCommand(
            FlywheelSubsystem shooter,
            KickerSubsystem kicker,
            IndexerSubsystem indexer,
            AgitatorSubsystem agitator,
            AngularVelocity goalRPM1            // <-- parameter passed in
    ) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.agitatorSubsystem = agitator;
        this.swerve = Optional.empty();
        this.goalRPM = goalRPM1;   // <-- store parameter

        addRequirements(kicker, indexer);
    }

    public ShootKickIndexCommand(
            FlywheelSubsystem shooter,
            KickerSubsystem kicker,
            IndexerSubsystem indexer,
            AgitatorSubsystem agitator,
            SwerveSubsystem swerve
    ) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.agitatorSubsystem = agitator;
        this.swerve = Optional.of(swerve);
        goalRPM = RPM.zero();

        for (var shot : shots) {
            calculatedGoalRPM.put(shot.distance.in(Meters), shot.shooterSpeed.in(RPM));
            calculatedTOF.put(shot.distance.in(Meters), shot.tof.in(Second));
        }
        addRequirements(kicker, indexer);
    }

    @Override
    public void initialize() {
        // Spin up shooter to the passed RPM
        kicker.setVelocitySetpoint(RPM.of(1000));
        if (swerve.isEmpty()) {
            shooter.setVelocitySetpoint(goalRPM);
        }
        agitatorSubsystem.setDutyCycleSetpoint(0);
    }

    @Override
    public void execute() {

        AngularVelocity goalRPM1 = goalRPM;
        if (swerve.isPresent()) {
            goalRPM1 = RPM.of(calculatedGoalRPM.get(swerve.get().distanceToHub()));
        }

        shooter.setVelocitySetpoint(goalRPM1);

        AngularVelocity shooterRPM = shooter.getRPM();

        boolean shooterReady = shootDebounce1.calculate(
                shooterRPM.isNear(
                        goalRPM1,
                        RPM.of(100)// tolerance
                )

        );

        if (shooterReady) {
            agitatorSubsystem.setDutyCycleSetpoint(0.5);
            indexer.setVelocitySetpoint(RPM.of(1000));
        } else {
            indexer.stop();
        }


    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        kicker.stop();
        indexer.stop();
        agitatorSubsystem.setDutyCycleSetpoint(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}