package frc.robot.subsystems.brain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefLevel;

public class SetLevelCmd extends Command {
    private BrainSubsystem brain_ ;
    private ReefLevel level_ ;

    public SetLevelCmd(BrainSubsystem brain, ReefLevel level) {
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
