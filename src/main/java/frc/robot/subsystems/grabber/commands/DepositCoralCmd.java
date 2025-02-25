package frc.robot.subsystems.grabber.commands;

import org.xerosw.util.XeroTimer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class DepositCoralCmd extends Command {

    private XeroTimer timer_ ;
    private GrabberSubsystem grabber_;
    private State state_;
    private ReefLevel level_ ;

    private enum State {
        WaitingForTimer,
        Finish
    }

    public DepositCoralCmd(GrabberSubsystem grabber, ReefLevel l1) {
        addRequirements(grabber);
        grabber_ = grabber;
        level_ = l1;
    }

    @Override
    public void initialize() {
        switch(level_) {
            case L1:
                grabber_.setGrabberMotorVoltage(6.0) ;
                timer_ = new XeroTimer(GrabberConstants.Grabber.DepositCoral.l1delay);
                timer_.start() ;
                state_ = State.WaitingForTimer ;
                break ;

            case L2:
            case L3:
                grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.DepositCoral.velocityl2l3) ;
                timer_ = new XeroTimer(GrabberConstants.Grabber.DepositCoral.delayl2l3);
                timer_.start() ;
                state_ = State.WaitingForTimer ;
                break ;

            case L4:
                grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.DepositCoral.velocityl4) ;
                timer_ = new XeroTimer(GrabberConstants.Grabber.DepositCoral.delayl4);
                timer_.start() ;
                state_ = State.WaitingForTimer ;
                break ;
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