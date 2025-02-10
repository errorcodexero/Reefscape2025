package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ExecuteClimbCmd extends Command {
    private ClimberSubsystem climber_;

    public ExecuteClimbCmd(ClimberSubsystem climber) {
        climber_ = climber;
        addRequirements(climber_);
    }
}
