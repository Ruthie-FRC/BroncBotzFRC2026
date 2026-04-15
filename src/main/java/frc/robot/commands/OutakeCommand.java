package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Setpoints;
import frc.robot.Setpoints.Intake;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

import static edu.wpi.first.units.Units.Degrees;

//import com.google.flatbuffers.Constants;
import frc.robot.Constants;


public class OutakeCommand extends Command {
    
    private final IntakeRollerSubsystem intakeRollerSubsystem;
    private final AgitatorSubsystem agitatorSubsystem;

    public OutakeCommand( IntakeRollerSubsystem intakeRollerSubsystem, AgitatorSubsystem agitatorSubsystem) {
       
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.agitatorSubsystem = agitatorSubsystem;
        addRequirements(this.intakeRollerSubsystem, this.agitatorSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        intakeRollerSubsystem.setDutycycleSetpoint(0);
        agitatorSubsystem.setDutyCycleCommand(0);
       
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
            intakeRollerSubsystem.setDutycycleSetpoint(Constants.IntakeRollerConstants.IntakeRollerOuttakeSpeeds);//(Setpoints.Intake.intakeRollerRPM);
            agitatorSubsystem.setDutyCycleCommand(Constants.AgitatorConstants.AgitatorInverseSpeed);
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
        return false;
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
        intakeRollerSubsystem.setDutycycleSetpoint(0);
    }
}
