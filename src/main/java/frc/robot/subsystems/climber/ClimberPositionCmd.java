package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPositionCmd extends Command {
    private ClimberState goalState_;
    private ClimberSubsystem sub_;

    public ClimberPositionCmd(ClimberSubsystem sub, ClimberState goal) {
        addRequirements(sub);

        sub_ = sub;
        goalState_ = goal;
    }

    @Override
    public void initialize() {
        sub_.setClimberTarget(goalState_.getAngle());
    }

    @Override
    public boolean isFinished() {
        boolean ret = false ;

        if (sub_.isClimberAtTarget()) {
            ret = true ;
            sub_.setClimberState(goalState_);
        }

        return ret;
    }
}