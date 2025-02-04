package frc.robot.commands.gps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.oi.OISubsystem;

public class AbortCmd extends Command {
    private OISubsystem oi_;

    public AbortCmd(OISubsystem oi) {
        oi_ = oi;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        oi_.clearRobotActions();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
