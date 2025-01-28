package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class GamepadEnabled extends Command {
    private boolean enabled_ ;

    public GamepadEnabled(boolean enabled) {
        enabled_ = enabled ;
    }

    @Override
    public void initialize() {
        RobotContainer c = RobotContainer.getRobotContainer() ;
        c.enableGamepad(enabled_) ;
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true ;
    }

    @Override
    public void end(boolean interrupted_) {
    }
}
