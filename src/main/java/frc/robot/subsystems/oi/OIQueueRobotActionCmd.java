package frc.robot.subsystems.oi;

import edu.wpi.first.wpilibj2.command.Command;

public class OIQueueRobotActionCmd extends Command {
    private OISubsystem oi_ ;
    private RobotAction action;

    public OIQueueRobotActionCmd(OISubsystem oi, RobotAction action) {
        this.action = action;
    }

    @Override
    public void initialize() {
        if (oi_.readyForAction()) {
            oi_.queueRobotAction(action);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true ;
    }

    @Override
    public void end(boolean interrupted) {
    }
}