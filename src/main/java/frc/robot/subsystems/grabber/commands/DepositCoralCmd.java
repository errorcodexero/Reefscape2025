package frc.robot.subsystems.grabber.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class DepositCoralCmd extends Command {

    private GrabberSubsystem grabber_;
    private Timer timer_ = new Timer();
    private State State_;

    private enum State {
        WaitingForCoralEject,
        RollersOff,
        Finish
    }

    public DepositCoralCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
    }

    @Override
    public void initialize() {
        grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.Positions.ejectCoralVelocty);
        State_ = State.WaitingForCoralEject;
    }

    @Override
    public boolean isFinished() {
        return State_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(State_){
            case WaitingForCoralEject:
                timer_.start();
                if (timer_.hasElapsed(GrabberConstants.Grabber.Positions.ejectCoralWait) && timer_.isRunning() && grabber_.coralFalling()) {
                    timer_.stop();
                    timer_.reset();
                    grabber_.setHasCoral(false);
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