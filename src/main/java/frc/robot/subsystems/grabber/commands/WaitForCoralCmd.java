package frc.robot.subsystems.grabber.commands;

import static edu.wpi.first.units.Units.Milliseconds;

import org.littletonrobotics.junction.Logger;
import org.xerosw.util.XeroTimer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class WaitForCoralCmd extends Command {

    private GrabberSubsystem grabber_;
    private State state_;
    private XeroTimer timer_ ;

    private enum State {
        WaitingForCoral,
        Delay,
        BackupCoral,
        Finish
    }

    public WaitForCoralCmd(GrabberSubsystem grabber) {
        addRequirements(grabber);
        grabber_ = grabber;
        timer_ = new XeroTimer(Milliseconds.of(200)) ;
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
        Logger.recordOutput("coralcollect-state", state_.toString()) ;
        switch(state_) {
            case WaitingForCoral:
                if (grabber_.coralFalling()) {
                    grabber_.setGrabberMotorVoltage(0.0) ;
                    timer_.start();
                    state_ = State.Delay;
                }
                break;

            case Delay:
                if (timer_.isExpired()) {
                    grabber_.setGrabberTargetVelocity(GrabberConstants.Grabber.CollectCoral.kBackupVelocity);
                    state_ = State.BackupCoral;
                }
                break ;

            case BackupCoral:
                if (grabber_.coralRising()) {
                    grabber_.setGrabberMotorVoltage(0.0) ;
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
