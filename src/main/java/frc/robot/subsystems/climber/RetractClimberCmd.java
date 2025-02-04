package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.FunnelSubsystem;

public class RetractClimberCmd extends Command {
    private ClimberSubsystem climber_ ;
    private FunnelSubsystem funnel_ ;

    private boolean retracting_climber_ ;

    public RetractClimberCmd(ClimberSubsystem climber, FunnelSubsystem funnel) {
        climber_ = climber ;
        funnel_ = funnel ;
    }

    @Override
    public void initialize() {
        retracting_climber_ = true ;
        climber_.setClimberPosition(ClimberConstants.Positions.kRetracted) ;
    }

    @Override
    public void execute() {
        if (retracting_climber_) {
            if (climber_.isAtTarget()) {
                retracting_climber_ = false ;
                funnel_.setTargetPosition(FunnelConstants.Positions.kUpPosition) ;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !retracting_climber_ && funnel_.isAtTarget() ;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
