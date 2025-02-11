package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class CollectAlgaeCmd extends Command {
    private GrabberSubsystem grabber_;
    private State state_;

    private enum State {
        WaitingForAlgae,
        Finish
    }

    public CollectAlgaeCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
    }

    @Override
    public void initialize() {
        grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.CollectAlgae.velocity);
        state_ = State.WaitingForAlgae;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(state_) {
            case WaitingForAlgae:
                if (grabber_.AlgaeRising()) {
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
