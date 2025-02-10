package frc.robot.subsystems.funnel;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class DeployFunnelCmd extends Command {
    private final FunnelSubsystem funnel_;
    private final Angle targetAngle_;
    private State state_;

    private enum State {
        MovingToPosition, 
        WaitingForCompletion,
        Done
    }

    public DeployFunnelCmd(FunnelSubsystem funnel, Angle targetAngle) {
        addRequirements(funnel);
        funnel_ = funnel;
        targetAngle_ = targetAngle;
    }

    @Override
    public void initialize() {
        funnel_.runPosition(targetAngle_);
        state_ = State.MovingToPosition;
    }

    @Override
    public void execute() {
        switch (state_) {
            case MovingToPosition:
                // check if the funnel has reached the target position
                if (hasReachedTarget()) {
                    state_ = State.WaitingForCompletion;
                }
                break;
            case WaitingForCompletion:
                // ensure the funnel is stable at the target position
                if (hasReachedTarget()) {
                    state_ = State.Done;
                }
                break;
            case Done:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return state_ == State.Done;
    }

    private boolean hasReachedTarget() {
        // Add logic to check if the funnel has reached the target position
        // This might involve checking the funnel position input
        return true; // Replace with actual condition
    }
}


