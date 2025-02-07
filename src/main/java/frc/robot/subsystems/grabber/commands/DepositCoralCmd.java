package frc.robot.subsystems.grabber.commands;

import org.xerosw.util.XeroTimer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class DepositCoralCmd extends Command {

    private XeroTimer timer_ ;
    private GrabberSubsystem grabber_;
    private State state_;

    private enum State {
        WaitingForCoralEject,
        WaitingForTimer,
        Finish
    }

    public DepositCoralCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
        timer_ = new XeroTimer(GrabberConstants.Grabber.DepositCoral.delay);
    }

    @Override
    public void initialize() {
        grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.DepositCoral.velocity) ;
        state_ = State.WaitingForCoralEject;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(state_){
            case WaitingForCoralEject:
                if (grabber_.coralRising()) {
                    timer_.start() ;
                    state_ = State.WaitingForTimer ;
                }
                break;
            case WaitingForTimer:
                if (timer_.isExpired()) {
                    grabber_.stopGrabber();
                    state_ = State.Finish;
                }
                break;
            case Finish:
                break;
        }
    }

    @Override
    public void end(boolean canceled) {
        state_ = State.Finish;
    }
}