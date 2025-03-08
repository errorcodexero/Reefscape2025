package frc.robot.subsystems.grabber.commands;

import static edu.wpi.first.units.Units.Milliseconds;

import org.littletonrobotics.junction.Logger;
import org.xerosw.util.XeroTimer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class WaitForCoralCmd extends Command {

    private static boolean showState = true ;

    private GrabberSubsystem grabber_;
    private State state_;
    private XeroTimer timer_ ;
    private boolean dobackup_ ;

    private enum State {
        WaitingForCoral,
        Delay,
        BackupCoral,
        Finish
    }

    public WaitForCoralCmd(GrabberSubsystem grabber, boolean dobackup) {
        addRequirements(grabber);
        grabber_ = grabber;
        timer_ = new XeroTimer(Milliseconds.of(40)) ;
        dobackup_ = dobackup ;
    }

    @Override
    public void initialize() {
        grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.CollectCoral.kVelocity);
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
                if (!grabber_.coralSensor()) {
                    grabber_.setGrabberMotorVoltage(0.0) ;
                    if (!dobackup_) {
                        state_ = State.Finish;
                    }
                    else {
                        timer_.start();
                        state_ = State.Delay;
                    }
                }
                break;

            case Delay:
                if (timer_.isExpired()) {
                    grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.CollectCoral.kBackupVelocity);
                    state_ = State.BackupCoral;
                }
                break ;

            case BackupCoral:
                if (grabber_.coralRising() || grabber_.coralSensor()) {
                    grabber_.setGrabberMotorVoltage(0.0) ;
                    state_ = State.Finish;
                }
                break;

            case Finish:
                break;
        }

        if (showState) {
            Logger.recordOutput("waitforcoral", state_.toString());
        }
    }

    @Override
    public void end(boolean canceled) {
        state_ = State.Finish;
    }
}
