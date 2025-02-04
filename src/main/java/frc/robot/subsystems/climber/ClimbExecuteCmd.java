package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbExecuteCmd extends Command {
    private ClimberSubsystem climber_ ;

    public ClimbExecuteCmd(ClimberSubsystem climber) {
        climber_ = climber ;
    }

    @Override
    public void initialize() {
        climber_.setClimberPosition(ClimberConstants.Positions.kClimbPosition) ;
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return climber_.isAtTarget() ;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
