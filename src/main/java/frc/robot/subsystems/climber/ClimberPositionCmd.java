package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPositionCmd extends Command {
    private ClimberTarget state_;
    private ClimberSubsystem sub_;
    private Angle target_angle_;
    private String position_;

    public enum ClimberTarget {
        Stowed,
        PrepareToClimb,
        Climb,
        Done
    }

    public ClimberPositionCmd(ClimberSubsystem sub, String position) {
        addRequirements(sub);
        position_ = position;
        sub_ = sub;

        state_ = ClimberTarget.Stowed;
    }

    @Override
    public void initialize() {
        // Everything else in here
        if (position_ == "Stowed") {
            target_angle_ = ClimberConstants.Climber.Position.kStowed;
            sub_.setClimberPosition(target_angle_);
            state_ = ClimberTarget.Stowed;

        } else if (position_ == "PrepareToClimb") {
            target_angle_ = ClimberConstants.Climber.Position.kPrepped;
            sub_.setClimberPosition(target_angle_);
            state_ = ClimberTarget.PrepareToClimb;

        } else if (position_ == "Climb") {
            target_angle_ = ClimberConstants.Climber.Position.kClimbed;
            sub_.setClimberPosition(target_angle_);
            state_ = ClimberTarget.Climb;
        }
    }

    @Override
    public void execute() {
        // Add switch and states in here
        switch (state_) {
            case Stowed:
                if (sub_.isClimberAtTarget()) {
                    state_ = ClimberTarget.Done;
                }
                break;
            case PrepareToClimb:
                if (sub_.isClimberAtTarget()) {
                    state_ = ClimberTarget.Done;
                }
                break;
            case Done:
                if (sub_.isClimberAtTarget()) {
                    state_ = ClimberTarget.Done;
                }
                break;
            case Climb:
                break;
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return state_ == ClimberTarget.Done;
    }

}
