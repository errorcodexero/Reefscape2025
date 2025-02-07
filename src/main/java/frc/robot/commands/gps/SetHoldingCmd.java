package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.GamePiece;
import frc.robot.subsystems.brain.Brain;

public class SetHoldingCmd extends Command {
    private Brain brain_ ;
    private GamePiece gp_ ;

    public SetHoldingCmd(Brain b, GamePiece gp) {
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
