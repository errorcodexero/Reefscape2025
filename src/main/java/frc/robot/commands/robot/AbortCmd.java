package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.brain.BrainSubsystem;

public class AbortCmd extends Command {
    private BrainSubsystem brain_ ;

    public AbortCmd(BrainSubsystem b) {
        brain_ = b ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        brain_.clearRobotActions();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
