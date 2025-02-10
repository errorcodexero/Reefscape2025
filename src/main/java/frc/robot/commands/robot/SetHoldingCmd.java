package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;
import frc.robot.subsystems.brain.GamePiece;

public class SetHoldingCmd extends Command {
    private BrainSubsystem brain_ ;
    private GamePiece gp_ ;

    public SetHoldingCmd(BrainSubsystem b, GamePiece gp) {
        brain_ = b ;
        gp_ = gp ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        brain_.setGp(gp_) ;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
