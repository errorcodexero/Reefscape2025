package frc.robot.subsystems.funnel;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public class DeployFunnelCmd extends Command {

    public enum Position {
        Normal,
        Climb
    } ;

    private final FunnelSubsystem funnel_;
    private final Position position_ ;


    public DeployFunnelCmd(FunnelSubsystem funnel, Position pos) {
        addRequirements(funnel);
        funnel_ = funnel;
        position_ = pos ;
    }

    @Override
    public void initialize() {
        Angle v = (position_ == Position.Normal) ? FunnelConstants.kNormalPosition : FunnelConstants.kClimbPosition ;
        funnel_.setTargetPosition(v) ;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return funnel_.isAtTarget() ;
    }
}
