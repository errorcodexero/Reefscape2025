package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitForCoralCmd extends Command {
    private enum State {
        WaitingForSensor,
        Backup,
        Done,
        Interrupted
    }

    private GrabberSubsystem grabber_ ;
    private State state_ ;

    public WaitForCoralCmd(GrabberSubsystem g) {
        addRequirements(g);

        grabber_ = g ;
    }   

    @Override
    public void initialize() {
        //
        // Turn on the roller motors
        //
        grabber_.setGrabberVelocity(GrabberConstants.Collect.kVelocity) ;
        state_ = State.WaitingForSensor ;
    }

    @Override
    public void execute() {
        switch(state_) {
            case WaitingForSensor:
                if (grabber_.isCoralSeenLowFallingEdge()) {
                    Angle a = grabber_.getPosition() ;
                    grabber_.setGrabberPosition(a.plus(GrabberConstants.Collect.kBackup)) ;
                    state_ = State.Backup ;
                }
                break ;
            case Backup:
                if (grabber_.isAtTarget()) {
                    grabber_.setGP(GamePieceLocation.Coral);
                    state_ = State.Done ;
                }
                break ;
            case Done:
            case Interrupted:
                break ;
        }
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done ;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            state_ = State.Interrupted ;
            grabber_.setGrabberVelocity(RevolutionsPerSecond.of(0.0)) ;
        }
    }
}
