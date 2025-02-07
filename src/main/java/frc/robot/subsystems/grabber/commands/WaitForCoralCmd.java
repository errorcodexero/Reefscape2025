package frc.robot.subsystems.grabber.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class WaitForCoralCmd extends Command {

    private GrabberSubsystem grabber_;
    private State state_;

    private enum State {
        WaitingForCoral,
        Finish
    }

    public WaitForCoralCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
    }

    @Override
    public void initialize() {
        grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.CollectCoral.velocity);
        state_ = State.WaitingForCoral;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(state_) {
            case WaitingForCoral:
                if (grabber_.coralFalling()) {
                    grabber_.stopGrabber();
                    state_ = State.Finish;
                }
                break;
            case Finish:
                break;
        }

        Logger.recordOutput("Grabber/WaitForCoral", state_.toString()) ;
    }

    @Override
    public void end(boolean canceled) {
        state_ = State.Finish;
    }
}
