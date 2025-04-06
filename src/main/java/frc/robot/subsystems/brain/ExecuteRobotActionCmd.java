package frc.robot.subsystems.brain;

import edu.wpi.first.wpilibj2.command.Command;

public class ExecuteRobotActionCmd extends Command {
    private BrainSubsystem exec_ ;

    public ExecuteRobotActionCmd(BrainSubsystem ex) {
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
