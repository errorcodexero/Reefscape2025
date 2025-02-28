package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class BackupCoralCmd extends Command {
    private final GrabberSubsystem grabber_;
    private boolean done_ ;

    public BackupCoralCmd(GrabberSubsystem grabberSubsystem) {
        this.grabber_ = grabberSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        if (grabber_.coralSensor()) {
            //
            // There is no coral hanging out of the grabber, do nothing
            //
            done_ = true ;
        }
        else {
            //
            // Run grabber backward until the coral is no longer detected
            //
            done_ = false ;
            grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.CollectCoral.kBackupVelocity);
        }
    }

    @Override
    public void execute() {
        if (grabber_.coralRising() || grabber_.coralSensor()) {
            grabber_.setGrabberMotorVoltage(0.0);
            done_ = true ;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            grabber_.setGrabberMotorVoltage(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return done_ ;
    }    
}
