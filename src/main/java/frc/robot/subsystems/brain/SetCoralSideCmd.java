package frc.robot.subsystems.brain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.oi.CoralSide;

public class SetCoralSideCmd extends Command {
    private BrainSubsystem brain_ ;
    private CoralSide side_ ;

    public SetCoralSideCmd(BrainSubsystem brain, CoralSide side) {
        side_ = side ;
        brain_ = brain ;
    }

    @Override
    public void initialize() {
        brain_.setCoralSide(side_) ;
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}
