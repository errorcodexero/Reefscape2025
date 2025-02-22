package frc.robot.commands.robot.climb;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberPositionCmd;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ExecuteClimbCmd extends XeroSequenceCmd {
    private ClimberSubsystem climber_;

    public ExecuteClimbCmd(ClimberSubsystem climber) {
        super("ExecuteClimbCmd");

        climber_ = climber;
    }

    @Override
    public void initSequence(SequentialCommandGroup sequence) {
        sequence.addCommands(new ClimberPositionCmd(climber_, ClimberState.Climb));
    }
}
