package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import java.util.List;
import java.util.Optional;

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


public class PassCommand extends Command {



    private final FlywheelSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;
    private final AgitatorSubsystem agitator;
    private final Optional<SwerveSubsystem> swerve;
    private final HoodSubsystem hood;//94 rpm
    private final AngularVelocity goalRPM;   // <-- parameter stored here
    //private final Angle goalDegree;

    private final Debouncer shootDebounce1 = new Debouncer(0.3, DebounceType.kFalling);

    
    private final InterpolatingDoubleTreeMap calculatedGoalRPM = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap calculatedTOF = new InterpolatingDoubleTreeMap();
   // private final InterpolatingDoubleTreeMap calculatedHoodAngle = new InterpolatingDoubleTreeMap();

    public PassCommand(
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

    // 30:8863
    // 40: 11817
    
   

    @Override
    public void initialize() {
        // Spin up shooter to the passed RPM
        kicker.setVelocitySetpoint(RPM.of(1513));
        hood.setAngleSetpoint(Setpoints.Intake.hoodPassAngle);
        if (swerve.isEmpty()) {
            shooter.setVelocitySetpoint(goalRPM);
            hood.setAngleSetpoint(Setpoints.Intake.hoodPassAngle);
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
        hood.setAngleSetpoint(Setpoints.Intake.hoodPassAngle);

        AngularVelocity shooterRPM = shooter.getRPM();

        boolean shooterReady = shootDebounce1.calculate(
                shooterRPM.isNear(
                        goalRPM1,
                        RPM.of(150)// tolerance
                )

        );
        
        kicker.setVelocitySetpoint(RPM.of(1792));

       agitator.setDutyCycleSetpoint(0.7);//RPM.of(1135)

        if (shooterReady) {

            indexer.setVelocitySetpoint(RPM.of(1707));
        } else {
            indexer.setDutyCycleSetpoint(-0.1);;
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