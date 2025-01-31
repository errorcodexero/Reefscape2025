package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.XeroTimer;

public class DepositAlgaeCmd extends Command {

    private GrabberSubsystem grabber_ ;
    private XeroTimer wait_timer_ ;
    private boolean done_ ;

    public DepositAlgaeCmd(GrabberSubsystem g) {
        addRequirements(g);

        grabber_ = g ;
        wait_timer_ = new XeroTimer(GrabberConstants.Place.kDelay) ;
    }   

    @Override
    public void initialize() {
        //
        // Turn on the roller motors
        //
        done_ = false ;
        grabber_.setGrabberVelocity(GrabberConstants.Place.kVelocity) ;
        wait_timer_.start() ;
    }

    @Override
    public void execute() {
        if (wait_timer_.isExpired()) {
            grabber_.setGrabberVelocity(RevolutionsPerSecond.of(0.0)) ;
            done_ = true ;
        }
    }

    @Override
    public boolean isFinished() {
        return done_ ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            grabber_.setGrabberVelocity(RevolutionsPerSecond.of(0.0)) ;
            done_ = true ;
        }
    }
}
