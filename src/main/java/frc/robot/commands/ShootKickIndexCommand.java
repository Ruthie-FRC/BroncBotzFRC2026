package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Map.Entry;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.*;
import frc.robot.subsystems.SwerveSubsystem;


public class ShootKickIndexCommand extends Command {

    private record RecordedShot(Distance distance, AngularVelocity shooterSpeed, Time tof) {
        public Pair<Double,Double> getRPM() { return Pair.of(distance.in(Meters), shooterSpeed.in(RPM));}
        public Pair<Double,Double> getTOF() { return Pair.of(distance.in(Meters), tof.in(Second));}
    }
    private final TurretFlywheelSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;
    private final Optional<SwerveSubsystem> swerve;

    private final double goalRPM;   // <-- parameter stored here

    private final Debouncer shootDebounce1 = new Debouncer(0.3, DebounceType.kFalling);

    private final List<RecordedShot> shots = List.of(
        // TUNE HERE
        new RecordedShot(Meters.of(1), RPM.of(4000), Second.of(1)),
        new RecordedShot(Meters.of(1), RPM.of(4000), Second.of(1))

    );
    private final InterpolatingDoubleTreeMap calculatedGoalRPM = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap calculatedTOF = new InterpolatingDoubleTreeMap();

    public ShootKickIndexCommand(
        TurretFlywheelSubsystem shooter,
        KickerSubsystem kicker,
        IndexerSubsystem indexer,
        double goalRPM1            // <-- parameter passed in
    ) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.swerve = Optional.empty();
        this.goalRPM = goalRPM1;   // <-- store parameter

        addRequirements(kicker, indexer);
    }

    public ShootKickIndexCommand(
        TurretFlywheelSubsystem shooter,
        KickerSubsystem kicker,
        IndexerSubsystem indexer, 
        SwerveSubsystem swerve
    ) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;
        this.swerve = Optional.of(swerve);
        goalRPM = 0;

        for(var shot : shots)
        {
            calculatedGoalRPM.put(shot.distance.in(Meters), shot.shooterSpeed.in(RPM));
            calculatedTOF.put(shot.distance.in(Meters), shot.tof.in(Second));
        }
        addRequirements(kicker, indexer);
    }

    @Override
    public void initialize() {
        // Spin up shooter to the passed RPM
    }

    @Override
    public void execute() {
        
        double goalRPM1 = goalRPM;
        if(swerve.isPresent())
        {
            goalRPM1 = swerve.get().distanceToHub();
        }

        shooter.setRPM(goalRPM1);

        AngularVelocity shooterRPM = shooter.getRPM();

        boolean shooterReady = shootDebounce1.calculate(
            shooterRPM.isNear(
                RPM.of(goalRPM1),
                RPM.of(100)// tolerance
            )

        );

          if(shooterReady){
            kicker.setRPM(0.75);
            indexer.setRPM(goalRPM);
        }else{
            kicker.setRPM(0.75);
            indexer.stop();
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        kicker.stop();
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}