package frc.robot.subsystems.brain;

import edu.wpi.first.wpilibj2.command.Command;

public class ExecuteRobotAction extends Command {
    private Brain exec_ ;

    public ExecuteRobotAction(Brain ex) {
        exec_ = ex ;
    }

    @Override
    public void initialize() {
        exec_.execute();
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
