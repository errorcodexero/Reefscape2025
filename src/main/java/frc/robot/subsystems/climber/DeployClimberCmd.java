package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.FunnelSubsystem;

public class DeployClimberCmd extends Command {
    private ClimberSubsystem climber_ ;
    private FunnelSubsystem funnel_ ;

    private boolean lowering_funnel_ ;

    public DeployClimberCmd(ClimberSubsystem climber, FunnelSubsystem funnel) {
        climber_ = climber ;
        funnel_ = funnel ;
    }

    @Override
    public void initialize() {
        lowering_funnel_ = true ;
        funnel_.setTargetPosition(FunnelConstants.Positions.kDownPosition) ;
    }

    @Override
    public void execute() {
        if (lowering_funnel_) {
            if (funnel_.isAtTarget()) {
                lowering_funnel_ = false ;
                climber_.setClimberPosition(ClimberConstants.Positions.kDeployed) ;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !lowering_funnel_ && climber_.isAtTarget() ;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
