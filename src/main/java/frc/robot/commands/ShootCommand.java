package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.io.LineNumberInputStream;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.PivotConstants;
import frc.robot.subsystems.Turret.HoodSubsystem;
import frc.robot.subsystems.Turret.IndexerSubsystem;
import frc.robot.subsystems.Turret.KickerSubsystem;
import frc.robot.subsystems.Turret.TurretFlywheelSubsystem;

public class ShootCommand extends Command{
    private final IndexerSubsystem indexer;
    private final KickerSubsystem kicker;
    private final HoodSubsystem hood;
    private final TurretFlywheelSubsystem flywheel;

    private final double goalRPM;
    
    public final Debouncer debouncer = new Debouncer(0.3, DebounceType.kFalling);

    public ShootCommand(IndexerSubsystem indexerSubsystem, KickerSubsystem kickerSubsystem, 
    HoodSubsystem hoodSubsystem, TurretFlywheelSubsystem turretFlywheelSubsystem, double goalRPM){
        this.indexer = indexerSubsystem;
        this.kicker = kickerSubsystem;
        this.hood = hoodSubsystem;
        this.flywheel = turretFlywheelSubsystem;
        this.goalRPM = goalRPM;
        addRequirements(indexer, kicker, hood, flywheel);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        flywheel.setRPM(goalRPM);
        AngularVelocity flywheelRPM = flywheel.getRPM();

        boolean flywheelReady = debouncer.calculate(
            flywheelRPM.isNear(
                RPM.of(goalRPM), 
                RPM.of(TurretConstants.flywheelTolerance)));

        if(flywheelReady){
            kicker.setRPM(0.75);
            indexer.setRPM(goalRPM);
        }else{
            kicker.setRPM(0.75);
            indexer.stop();
        }
    }

    @Override
    public void end(boolean Interrupted){
        flywheel.stop();
        kicker.stop();
        indexer.stop();

    }
    @Override 
    public boolean isFinished(){
        return false;
    }
}
