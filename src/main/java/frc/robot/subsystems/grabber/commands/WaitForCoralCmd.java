package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class WaitForCoralCmd extends Command {

    private GrabberSubsystem grabber_;
    private State State_;

    private enum State {
        WaitingForCoral,
        RollersOff,
        Position,
        Finish
    }

    public WaitForCoralCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
    }

    @Override
    public void initialize() {
        grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.Positions.waitForCoralVelocity);
        State_ = State.WaitingForCoral;
    }

    @Override
    public boolean isFinished() {
        return State_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(State_) {
            case WaitingForCoral:
                if (grabber_.coralRising()) {
                    grabber_.setHasCoral(true);
                    State_ = State.RollersOff;
                }
                break;
            case RollersOff:
                grabber_.stopGrabber();
                State_ = State.Position;
                break;
            case Position:
                grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.Positions.CoralPositionVelocity);
                if (grabber_.coralFalling()) {
                    grabber_.stopGrabber();
                    State_ = State.Finish;
                }
            case Finish:
                break;
        }
    }

    @Override
    public void end(boolean canceled) {
        State_ = State.Finish;
    }
}
