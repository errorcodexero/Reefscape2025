package frc.robot.subsystems.brain;

import edu.wpi.first.wpilibj2.command.Command;

public class QueueRobotActionCmd extends Command {
    private BrainSubsystem exec_ ;
    private RobotAction action_;

    public QueueRobotActionCmd(BrainSubsystem ex, RobotAction action) {
        exec_ = ex ;
        action_ = action;
    }

    @Override
    public void initialize() {
        if (exec_.readyForAction()) {
            exec_.queueRobotAction(action_);
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
