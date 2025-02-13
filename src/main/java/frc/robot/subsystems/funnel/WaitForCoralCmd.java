package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForCoralCmd extends Command {
    private final FunnelSubsystem funnel_;

    public WaitForCoralCmd (FunnelSubsystem funnel) {
        addRequirements(funnel);
        funnel_ = funnel;
    }

    @Override
    public boolean isFinished() {
        return funnel_.hasSeenCoralWithReset();
    }
}