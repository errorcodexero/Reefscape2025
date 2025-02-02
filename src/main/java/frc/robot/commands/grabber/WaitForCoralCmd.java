package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class WaitForCoralCmd extends Command {

    private GrabberSubsystem grabber_;
    private WaitForCoralCmdState WaitForCoralCmdState_;

    private enum WaitForCoralCmdState {
        WaitingForCoral,
        RollersOff,
        Finish
    }

    public WaitForCoralCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
    }

    @Override
    public void initialize() {
        grabber_.stopGrabber();
    }

    @Override
    public boolean isFinished() {
        return WaitForCoralCmdState_ == WaitForCoralCmdState.Finish;
    }

    @Override
    public void execute() {
        switch(WaitForCoralCmdState_) {
            case WaitingForCoral:
                grabber_.waitForCoral();
                WaitForCoralCmdState_ = WaitForCoralCmdState.RollersOff;
                break;
            case RollersOff:
                grabber_.holdingCoral();
                WaitForCoralCmdState_ = WaitForCoralCmdState.Finish;
                break;
            case Finish:
                break;
        }
    }

    @Override
    public void end(boolean canceled) {
        WaitForCoralCmdState_ = WaitForCoralCmdState.Finish;
    }
}
