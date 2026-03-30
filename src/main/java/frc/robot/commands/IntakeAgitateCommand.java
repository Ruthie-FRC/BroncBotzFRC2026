package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Setpoints;
import frc.robot.Setpoints.Intake;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.Timer;


public class IntakeAgitateCommand extends Command {
    private final IntakeArmSubsystem IntakeArmSubsystem;
    Timer timer = new Timer();
    Timer timer2 = new Timer();
    //private final HoodSubsystem hoodSubsystem;

    public IntakeAgitateCommand(IntakeArmSubsystem IntakeArmSubsystem
    //HoodSubsystem hoodSubsystem
    ) {
        //this.intakeArmSubsystem = intakeArmSubsystem;
        this.IntakeArmSubsystem = IntakeArmSubsystem;
        //this.hoodSubsystem = hoodSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.IntakeArmSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        IntakeArmSubsystem.setAngleSetpoint(Setpoints.Intake.intakeArmAngleDown);
        timer.reset();
       timer.start();

        
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        //intakeArmSubsystem.setAngleSetpoint(Intake.intakeArmAngleDown);
       // hoodSubsystem.setAngleSetpoint(Intake.hoodDownAngle);

        if (timer.hasElapsed(0.3)){
         IntakeArmSubsystem.setAngleSetpoint(Setpoints.Intake.intakeArmAngleUp);
        timer.reset();
        timer.start();
        } 
  
        
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     *
     */
    @Override
    public boolean isFinished() {
        
        // TODO: Make this return true when this Command no longer needs to run execute()
        return  IntakeArmSubsystem.getAngle().isNear(Setpoints.Intake.intakeArmAngleUp, Degrees.of(4));
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        timer.reset();
        timer.start();
        if (timer.hasElapsed(0.3)){
         IntakeArmSubsystem.setAngleSetpoint(Setpoints.Intake.intakeArmAngleDown);
        } 
        
    }
}
