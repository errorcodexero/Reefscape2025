package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class CollectAlgaeCmd extends Command {

    private GrabberSubsystem grabber_;
    private State State_;

    private enum State {
        WaitingForAlgae,
        RollersOff,
        Finish
    }

    public CollectAlgaeCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
    }
    @Override
    public void initialize() {
        grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.Positions.collectAlgaeVelocity);
        State_ = State.WaitingForAlgae;
    }

    @Override
    public boolean isFinished() {
        return State_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(State_) {
            case WaitingForAlgae:
                if (grabber_.AlgaeRising()) {
                    grabber_.setHasAlgae(true);
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
