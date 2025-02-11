package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPositionCmd extends Command {

    private ClimberTarget goalState_;
    private ClimberSubsystem sub_;

    
    public enum ClimberTarget {
        Stowed(ClimberConstants.Climber.Position.kStowed),
        PrepareToClimb(ClimberConstants.Climber.Position.kPrepped),
        Climb(ClimberConstants.Climber.Position.kClimbed);

        private final Angle angle;

        private ClimberTarget(Angle angle) {
            this.angle = angle;
        }

        public Angle getAngle() {
            return angle;
        }
    }

    public ClimberPositionCmd(ClimberSubsystem sub, ClimberTarget goal) {
        addRequirements(sub);

        sub_ = sub;
        goalState_ = goal;
    }

    @Override
    public void initialize() {
        sub_.setClimberPosition(goalState_.getAngle());
    }

    @Override
    public boolean isFinished() {
        return sub_.isClimberAtTarget();
    }

}