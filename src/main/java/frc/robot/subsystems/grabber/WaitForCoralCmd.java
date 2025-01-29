package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.XeroTimer;

public class WaitForCoralCmd extends Command {
    private enum State {
        WaitingForSensor,
        WaitingForDelay,
        Done,
        Interrupted
    }

    private GrabberSubsystem grabber_ ;
    private XeroTimer wait_timer_ ;
    private State state_ ;

    public WaitForCoralCmd(GrabberSubsystem g) {
        addRequirements(g);

        grabber_ = g ;
        wait_timer_ = new XeroTimer(GrabberConstants.Collect.kDelay) ;
    }   

    @Override
    public void initialize() {
        //
        // Turn on the roller motors
        //
        grabber_.setGrabberVelocity(GrabberConstants.Collect.kVelocity) ;
        state_ = State.WaitingForSensor ;
    }

    @Override
    public void execute() {
        switch(state_) {
            case WaitingForSensor:
                if (grabber_.isCoralSeen()) {
                    wait_timer_.start() ;
                    state_ = State.WaitingForDelay ;
                }
                break ;
            case WaitingForDelay:
                if (wait_timer_.isExpired()) {
                    grabber_.setGrabberVelocity(RevolutionsPerSecond.of(0.0)) ;
                    state_ = State.Done ;
                }
                break ;
            case Done:
            case Interrupted:
                break ;
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            state_ = State.Interrupted ;
            grabber_.setGrabberVelocity(RevolutionsPerSecond.of(0.0)) ;
        }
    }
}
