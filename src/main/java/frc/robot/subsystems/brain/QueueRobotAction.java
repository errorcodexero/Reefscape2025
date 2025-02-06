package frc.robot.subsystems.brain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.oi.RobotAction;

public class QueueRobotAction extends Command {
    private Brain exec_ ;
    private RobotAction action_;

    public QueueRobotAction(Brain ex, RobotAction action) {
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
