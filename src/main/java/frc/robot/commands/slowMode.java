package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.systems.field.*;
import frc.robot.systems.field.FieldConstants.Hub;
import swervelib.SwerveInputStream;


public class slowMode extends Command
{

  private final SwerveSubsystem   swerveSubsystem;
  private final SwerveInputStream swerveInputStream;

  public slowMode(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.copy();
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize()
  {
    swerveInputStream
                     .scaleTranslation(0.25);

  }

  @Override
  public void execute()
  {
    swerveSubsystem.driveFieldOrientedSetpoint(swerveInputStream.get());
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    swerveInputStream
                     .scaleTranslation(1);
  }
}