package frc.robot.subsystems.brain;

import edu.wpi.first.wpilibj2.command.Command;

public class SetLevelCmd extends Command {
    private BrainSubsystem brain_ ;
    private int level_ ;

    public SetLevelCmd(BrainSubsystem brain, int level) {
        level_ = level ;
        brain_ = brain ;
    }

    @Override
    public void initialize() {
        brain_.setCoralLevel(level_) ;
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}
