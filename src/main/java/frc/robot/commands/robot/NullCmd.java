package frc.robot.commands.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class NullCmd extends Command {
    public NullCmd() {
        super();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}
