package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class WaitForCoralInRobot extends Command {
    private GrabberSubsystem g_ ;
    private FunnelSubsystem f_ ;
    private boolean seen_ ;

    public WaitForCoralInRobot(GrabberSubsystem grabber, FunnelSubsystem funnel) {
        g_ = grabber ;
        f_ = funnel ;
    }

    @Override
    public void initialize() {
        seen_ = false ;
    }

    @Override
    public void execute() {
        if (g_.hasSeenCoral() || f_.hasSeenCoral()) {
            seen_ = true ;
        }
    }

    @Override
    public boolean isFinished() {
        return seen_ ;
    }
}
