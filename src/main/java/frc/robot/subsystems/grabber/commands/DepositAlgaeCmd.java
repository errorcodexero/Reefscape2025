package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class DepositAlgaeCmd extends Command {

    private GrabberSubsystem grabber_;
    private State State_;
    private Timer timer_;

    private enum State {
        WaitingForAlgaeEject,
        RollersOff,
        Finish
    }

    public DepositAlgaeCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
    }
    @Override
    public void initialize() {
        grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.Positions.waitForCoralVelocity);
        State_ = State.WaitingForAlgaeEject;
    }

    @Override
    public boolean isFinished() {
        return State_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(State_) {
            case WaitingForAlgaeEject:
                timer_.start();
                if (grabber_.AlgaeFalling() && timer_.hasElapsed(GrabberConstants.Grabber.Positions.ejectAlgaeWait) && timer_.isRunning()) {
                    grabber_.setHasAlgae(false);
                    timer_.stop();
                    timer_.reset();
                    State_ = State.RollersOff;
                }
                break;
            case RollersOff:
                grabber_.stopGrabber();
                State_ = State.Finish;
                break;
            case Finish:
                break;
        }
    }

    @Override
    public void end(boolean canceled) {
        State_ = State.Finish;
    }
}
