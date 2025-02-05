package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj2.command.Command;

public class OIQueueRobotActionCmd extends Command {
    private OISubsystem oi_ ;
    private RobotAction action_;

    public OIQueueRobotActionCmd(OISubsystem oi, RobotAction action) {
        oi_ = oi ;
        action_ = action;
    }

    @Override
    public void initialize() {
        if (oi_.readyForAction()) {
            oi_.queueRobotAction(action_);
        }
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

    public String getName() {
        return "OIQueueRobotActionCmd:" + action_.toString() ;
    }
}
