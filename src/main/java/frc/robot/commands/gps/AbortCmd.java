package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AbortCmd extends Command {
    public AbortCmd() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.getRobotContainer().getExecutor().clearRobotActions() ;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
