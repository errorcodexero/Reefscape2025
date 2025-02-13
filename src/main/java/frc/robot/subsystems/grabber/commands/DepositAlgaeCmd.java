package frc.robot.subsystems.grabber.commands;

import org.xerosw.util.XeroTimer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class DepositAlgaeCmd extends Command {

    private GrabberSubsystem grabber_;
    private State state_;
    private XeroTimer timer_;

    private enum State {
        WaitForTimer,
        Finish
    }

    public DepositAlgaeCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
        timer_ = new XeroTimer(GrabberConstants.Grabber.DepositAlgae.delay);
    }

    @Override
    public void initialize() {
        grabber_.setGrabberMotorVoltage(-10);
        state_ = State.WaitForTimer;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(state_) {
            case WaitForTimer:
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
