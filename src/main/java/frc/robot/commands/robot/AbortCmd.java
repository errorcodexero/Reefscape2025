package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;

public class AbortCmd extends Command {
    private BrainSubsystem b_ ;

    public AbortCmd(BrainSubsystem b) {
        b_ = b ;
    }

    @Override
    public void initialize() {
        b_.clearRobotActions() ;
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}
