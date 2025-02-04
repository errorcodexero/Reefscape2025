package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;

public class DropFunnelCmd extends Command {
    private FunnelSubsystem funnel_ ;

    public DropFunnelCmd(FunnelSubsystem funnel) {
        funnel_ = funnel ;
    }

    @Override
    public void initialize() {
        funnel_.setTargetPosition(FunnelConstants.Positions.kDownPosition) ;
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true ;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
