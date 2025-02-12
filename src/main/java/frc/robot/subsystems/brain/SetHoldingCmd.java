package frc.robot.subsystems.brain;

import edu.wpi.first.wpilibj2.command.Command;

public class SetHoldingCmd extends Command {
    private BrainSubsystem brain_ ;
    private GamePiece gp_ ;

    public SetHoldingCmd(BrainSubsystem brain, GamePiece gp) {
        brain_ = brain ;
        gp_ = gp ;
    }

    @Override
    public void initialize() {
        brain_.setGp(gp_) ;
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}
