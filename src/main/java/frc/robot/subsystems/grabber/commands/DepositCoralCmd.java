package frc.robot.subsystems.grabber.commands;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import org.xerosw.util.XeroTimer;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.grabber.GrabberConstants;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class DepositCoralCmd extends Command {

    private XeroTimer timer_ ;
    private GrabberSubsystem grabber_;
    private State state_;
    private ReefLevel level_ ;

    private enum State {
        WaitingForTimer,
        Finish
    }

    public DepositCoralCmd(GrabberSubsystem grabber, ReefLevel level) {
        addRequirements(grabber);
        grabber_ = grabber;
        level_ = level;
    }

    @Override
    public void initialize() {
        if (level_ == ReefLevel.L1) {
            grabber_.idle() ;
            grabber_.setGrabberMotorVoltage(Volts.of(6.0)) ;
            timer_ = new XeroTimer(GrabberConstants.Grabber.DepositCoral.l1delay);
            timer_.start() ;
            state_ = State.WaitingForTimer ;
        }
        else {
            grabber_.idle() ;
            grabber_.setGrabberMotorVoltage(Volts.of(12.0)) ;
            timer_ = new XeroTimer(depDelay()) ;
            timer_.start() ;
            state_ = State.WaitingForTimer ;
        }
    }

    private Time depDelay() {
        return GrabberConstants.Grabber.DepositCoral.delay ;
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Finish;
    }

    @Override
    public void execute() {
        switch(state_){
            case WaitingForTimer:
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