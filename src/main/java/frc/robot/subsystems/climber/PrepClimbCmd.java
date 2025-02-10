package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class PrepClimbCmd extends SequentialCommandGroup {
    private ClimberSubsystem climber_;

    public PrepClimbCmd(ClimberSubsystem climber) {
        climber_ = climber;
        addRequirements(climber_);
    }
}

