package frc.robot.subsystems.grabber.commands;

import org.xerosw.util.XeroTimer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class DepositCoralCmd extends Command {

    private XeroTimer timer_ ;
    private GrabberSubsystem grabber_;
    private State state_;
    private boolean l1_ ;

    private enum State {
        WaitingForTimer,
        Finish
    }

    public DepositCoralCmd(GrabberSubsystem grabber) {
        this(grabber, false) ;
    }

    public DepositCoralCmd(GrabberSubsystem grabber, boolean l1) {
        addRequirements(grabber);
        grabber_ = grabber;

        l1_ = l1 ;
    }

    @Override
    public void initialize() {
        if (l1_) {
            grabber_.setGrabberMotorVoltage(6.0) ;
            timer_ = new XeroTimer(GrabberConstants.Grabber.DepositCoral.l1delay);
            timer_.start() ;
            state_ = State.WaitingForTimer ;
        }
        else {
            grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.DepositCoral.velocity) ;
            timer_ = new XeroTimer(GrabberConstants.Grabber.DepositCoral.delay);
            timer_.start() ;
            state_ = State.WaitingForTimer ;
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(state_){
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