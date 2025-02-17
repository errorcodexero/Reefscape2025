package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.brain.BrainSubsystem;

public class AbortCmd extends Command {
    private BrainSubsystem b_ ;

    public AbortCmd(BrainSubsystem b) {
        b_ = b ;
    }

    @Override
    public void initialize() {
        b_.lock() ;
        b_.clearRobotActions() ;
        b_.unlock() ;
        RobotContainer.getInstance().gamepad().setLocked(false) ;
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}
