package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DepositCoralCmd extends Command {

    private GrabberSubsystem grabber_;
    private Timer timer_ = new Timer();
    private DepositCoralCmdState DepositCoralCmdState_;

    private enum DepositCoralCmdState {
        RollersOn,
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
        grabber_.stopGrabber();
    }

    @Override
    public boolean isFinished() {
        return DepositCoralCmdState_ == DepositCoralCmdState.Finish;
    }

    @Override
    public void execute() {
        switch(DepositCoralCmdState_){
            case RollersOn:
                grabber_.ejectCoralReef();
                DepositCoralCmdState_ = DepositCoralCmdState.WaitingForCoralEject;
                break;
            case WaitingForCoralEject:
                timer_.start();
                if (timer_.hasElapsed(GrabberConstants.Grabber.Positions.ejectCoralWait) && timer_.isRunning() && !grabber_.hasCoral()) {
                    timer_.stop();
                    timer_.reset();
                    DepositCoralCmdState_ = DepositCoralCmdState.RollersOff;
                }
                break;
            case RollersOff:
                grabber_.stopGrabber();
                DepositCoralCmdState_ = DepositCoralCmdState.Finish;
                break;
            case Finish:
                break;
        }
    }

    @Override
    public void end(boolean canceled) {
        DepositCoralCmdState_ = DepositCoralCmdState.Finish;
    }
}
