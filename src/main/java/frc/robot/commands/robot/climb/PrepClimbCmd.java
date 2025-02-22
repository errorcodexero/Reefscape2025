package frc.robot.commands.robot.climb;

import org.xerosw.util.XeroSequenceCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberPositionCmd;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.funnel.DeployFunnelCmd;
import frc.robot.subsystems.funnel.FunnelSubsystem;

public class PrepClimbCmd extends XeroSequenceCmd {
    private ClimberSubsystem climber_ ;
    private FunnelSubsystem funnel_ ;

    public PrepClimbCmd(ClimberSubsystem climber, FunnelSubsystem funnel) {
        super("PrepClimbCmd");

        climber_ = climber ;
        funnel_ = funnel ;
    }

    @Override
    public void initSequence(SequentialCommandGroup sequence) {
        sequence.addCommands(new DeployFunnelCmd(funnel_, ClimbConstants.Funnel.kClimbAngle)) ;
        sequence.addCommands(new ClimberPositionCmd(climber_, ClimberState.PrepareToClimb)) ;
    }
}
